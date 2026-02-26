import sys
sys.path.insert(0, '/usr/lib/python3/dist-packages')
sys.path.insert(0, '/usr/local/lib/python3.12/dist-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
TARGET_CLASSES = ["Kirmizi_Ucgen", "Mavi_Altigen"]
PIXEL_TOLERANCE = 40

SEARCHING = "SEARCHING"
CENTERING = "CENTERING"
LANDING   = "LANDING"

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.bridge = CvBridge()
        self.model = YOLO("/home/adem/Desktop/best.pt")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        self.current_pose = PoseStamped()
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)

        self.setpoint_pub    = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.target_pub      = self.create_publisher(Bool, '/target_detected', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.state           = SEARCHING
        self.target_class    = None
        self.hover_pose      = None
        self.target_pose     = None
        self.centered_since  = None
        self.CENTER_HOLD_TIME = 2.0

        self.create_timer(0.05, self.publish_setpoint)
        self.get_logger().info("YOLO Detector baslatildi!")

    def pose_callback(self, msg):
        self.current_pose = msg

    def publish_setpoint(self):
        if self.state == LANDING:
            return
        if self.state == CENTERING and self.target_pose is not None:
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target_pose)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
        self.get_logger().info(f"Mod: {mode}")

    def image_callback(self, msg):
        if self.state == LANDING:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w  = frame.shape[:2]
        img_cx, img_cy = w // 2, h // 2

        results   = self.model(frame, conf=0.75)
        annotated = results[0].plot()

        cv2.circle(annotated, (img_cx, img_cy), 8, (255, 255, 0), -1)
        cv2.putText(annotated, f"Durum: {self.state}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        hedef_bulundu = False

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            label  = self.model.names[cls_id]
            conf   = float(box.conf[0])

            if label not in TARGET_CLASSES:
                continue

            if self.state == SEARCHING:
                self.state        = CENTERING
                self.target_class = label
                self.get_logger().info(f"HEDEF KILITLENDI: {label} ({conf:.2f})")
                signal = Bool()
                signal.data = True
                self.target_pub.publish(signal)

            if label != self.target_class:
                continue

            hedef_bulundu = True

            x1, y1, x2, y2 = box.xyxy[0]
            obj_cx = int((x1 + x2) / 2)
            obj_cy = int((y1 + y2) / 2)

            err_x = obj_cx - img_cx
            err_y = obj_cy - img_cy

            cv2.line(annotated, (img_cx, img_cy), (obj_cx, obj_cy), (0, 255, 0), 2)
            cv2.putText(annotated, f"Hata: ({err_x}, {err_y})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            self.get_logger().info(f"[{self.state}] {label} | Hata: ({err_x}, {err_y})")

            if self.state == CENTERING:
                if abs(err_x) < PIXEL_TOLERANCE and abs(err_y) < PIXEL_TOLERANCE:
                    if self.centered_since is None:
                        self.centered_since = self.get_clock().now().seconds_nanoseconds()[0]
                        self.get_logger().info("Merkeze girdi, 2sn bekleniyor...")
                    else:
                        now     = self.get_clock().now().seconds_nanoseconds()[0]
                        elapsed = now - self.centered_since
                        self.get_logger().info(f"Merkezde: {elapsed:.1f}s / {self.CENTER_HOLD_TIME}s")
                        if elapsed >= self.CENTER_HOLD_TIME:
                            self.get_logger().info("MERKEZ ONAYLANDI → INIS BASLIYOR!")
                            self.state = LANDING
                            self.target_pose = None
                            self.set_mode('LAND')
                else:
                    self.centered_since = None
                    self.move_towards_target(err_x, err_y)

        if self.state == CENTERING and not hedef_bulundu:
            self.get_logger().info("Hedef kayboldu, hover ediliyor...")
            self.centered_since = None
            self.hover_at_current()

        cv2.imshow("YOLO - Drone Kamera", annotated)
        cv2.waitKey(1)

    def hover_at_current(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.current_pose.pose.position.x
        pose.pose.position.y = self.current_pose.pose.position.y
        pose.pose.position.z = self.current_pose.pose.position.z
        pose.pose.orientation.w = 1.0
        self.target_pose = pose

    def move_towards_target(self, err_x, err_y):
        scale = 0.02

        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z
        new_x = cx - err_y * scale  # görüntü yukarı-aşağı → drone ileri-geri
        new_y = cy - err_x * scale  # görüntü sağ-sol → drone sağ-sol
        new_z = cz

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = new_x
        pose.pose.position.y = new_y
        pose.pose.position.z = new_z
        pose.pose.orientation.w = 1.0
        self.target_pose = pose


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()