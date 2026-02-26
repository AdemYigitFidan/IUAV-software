#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Bool
import time
import math

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')  
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        self.state_sub     = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_pos_callback, qos_profile)
        self.target_sub    = self.create_subscription(Bool, '/target_detected', self.target_callback, 10)
        
        self.arming_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client  = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        self.current_state = State()
        self.current_pose  = PoseStamped()
        self.target_pose   = PoseStamped()
        
        self.waypoints_completed = 0
        self.total_waypoints     = 0
        self.target_detected     = False
        self.hover_pose          = None  # Hedef bulununca bu konumda kal

        # 20Hz setpoint yayını
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.log_info('='*50)
        self.log_info('Drone Controller Baslatildi')
        self.log_info('='*50)

    def log_info(self, msg): self.get_logger().info(msg)
    def log_warn(self, msg): self.get_logger().warn(msg)
    def log_error(self, msg): self.get_logger().error(msg)

    def state_callback(self, msg):
        self.current_state = msg
        
    def local_pos_callback(self, msg):
        self.current_pose = msg

    def target_callback(self, msg):
        if msg.data and not self.target_detected:
            self.target_detected = True
            self.hover_pose = PoseStamped()
            self.hover_pose.header.frame_id = "map"
            self.hover_pose.pose.position.x = self.current_pose.pose.position.x
            self.hover_pose.pose.position.y = self.current_pose.pose.position.y
            self.hover_pose.pose.position.z = self.current_pose.pose.position.z
            self.hover_pose.pose.orientation.w = 1.0
            self.target_pose = self.hover_pose
            self.log_info(f"YOLO hedef buldu! Hover konumu: ({self.hover_pose.pose.position.x:.1f}, {self.hover_pose.pose.position.y:.1f}, {self.hover_pose.pose.position.z:.1f})")

    def timer_callback(self): #Interrupting the process
        if self.target_detected:
            return
        if hasattr(self, 'target_pose') and self.target_pose.header.frame_id == "map":
            self.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.local_pos_pub.publish(self.target_pose)

    def get_pos(self):
        p = self.current_pose.pose.position
        return p.x, p.y, p.z

    def distance_to(self, x, y, z):
        px, py, pz = self.get_pos()
        return math.sqrt((px-x)**2 + (py-y)**2 + (pz-z)**2)

    def set_mode(self, mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.log_error('set_mode servisi bulunamadi!')
            return False
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None and future.result().mode_sent:
            self.log_info(f'Mod degisti: {mode}')
            return True
        self.log_error(f'Mod degistirilemedi: {mode}')
        return False

    def arm(self, max_retries=5, retry_delay=3.0): # arm etmek için 5 kere 3 saniye arayla deneme
        for attempt in range(1, max_retries+1):
            self.log_info('Motorlar arm ediliyor...')
            if not self.arming_client.wait_for_service(timeout_sec=5.0):
                self.log_error('Arming servisi bulunamadi!')
                return False
            request = CommandBool.Request()
            request.value = True
            future = self.arming_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None and future.result().success:
                self.log_info('Motorlar ARM edildi!')
                return True
            self.log_error(f'ARM edilemedi -- {attempt}/{max_retries}')
            if attempt < max_retries:
                time.sleep(retry_delay)
        return False

    def takeoff(self, altitude):
        self.log_info(f'KALKIS -> {altitude}m')
        if not self.set_mode('GUIDED'):
            return False
        time.sleep(1)
        if not self.arm():
            return False
        time.sleep(1)

        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            return False
        request = CommandTOL.Request()
        request.altitude = float(altitude)
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            start = time.time()
            while rclpy.ok():
                _, _, z = self.get_pos()
                self.log_info(f'Irtifa: {z:.2f}m / {altitude}m')
                if z >= altitude * 0.85:
                    self.log_info('Kalkis tamamlandi!')
                    return True
                if time.time() - start > 20.0:
                    return False
                time.sleep(1.0)
                rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def goto_position(self, x, y, z, tolerance=0.5, timeout=40.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        self.target_pose = pose

        start_time = time.time()
        last_log   = 0

        while rclpy.ok():
            if self.target_detected:
                self.log_info("Hedef tespit edildi, duruluyor!")
                return False

            dist    = self.distance_to(x, y, z)
            elapsed = time.time() - start_time

            if elapsed - last_log >= 3.0:
                px, py, pz = self.get_pos()
                self.log_info(f'Konum: ({px:.1f},{py:.1f},{pz:.1f}) -> ({x:.1f},{y:.1f},{z:.1f}) | {dist:.2f}m')
                last_log = elapsed

            if dist < tolerance:
                return True
            if elapsed > timeout:
                return False

            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def move_to_waypoint(self, x, y, z, label="", hold_duration=1.0):
        if self.target_detected:
            return False

        self.waypoints_completed += 1
        progress = f'[{self.waypoints_completed}/{self.total_waypoints}]'
        self.log_info(f'{progress} {label}: ({x:.1f}, {y:.1f}, {z:.1f})')
        
        success = self.goto_position(x, y, z)
        
        if success and not self.target_detected:
            if hold_duration > 0:
                time.sleep(hold_duration)
        return success

    def land(self):
        self.log_info('INIS BASLIYOR...')
        return self.set_mode('LAND')


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()

    try:
        input('\nEnter Bas\n')

        if not controller.takeoff(5):
            controller.log_error('Kalkis basarisiz oldu')
            return
        time.sleep(3)

        area_size = 18.0
        step_size = 3.0
        altitude  = 5.0
        limit     = area_size / 2.0

        #lawn-mower algorithm

        y_points = []
        curr_y = -limit
        while curr_y <= limit + 0.01:
            y_points.append(round(curr_y, 2))
            curr_y += step_size

        controller.total_waypoints = len(y_points) * 2 + 1
        controller.log_info(f'Yilan tarama: {area_size}x{area_size}m | {len(y_points)} satir')

        mission_start = time.time()

        for i, y_val in enumerate(y_points):
            if controller.target_detected:
                controller.log_info(f"Satir {i+1} basinda hedef bulundu! YOLO devralıyor...")
                break

            x_start = -limit if i % 2 == 0 else limit
            x_end   =  limit if i % 2 == 0 else -limit

            controller.log_info(f'--- Satir {i+1}/{len(y_points)} | Y={y_val}m ---')
            controller.move_to_waypoint(x_start, y_val, altitude, f'Satir {i+1} Basi')
            controller.move_to_waypoint(x_end,   y_val, altitude, f'Satir {i+1} Sonu')

        if not controller.target_detected:
            controller.log_info('Tarama bitti, eve donuluyor...')
            controller.move_to_waypoint(0.0, 0.0, altitude, 'Home')
            controller.log_info(f'GOREV TAMAMLANDI! Sure: {time.time()-mission_start:.1f}s')
            controller.land()
        else:
            controller.log_info("YOLO inis yapiyor, bekleniyor...")
            # YOLO iniş yapana kadar bekle
            while rclpy.ok():
                rclpy.spin_once(controller, timeout_sec=0.1)

    except KeyboardInterrupt:
        controller.log_warn('Durduruldu!')
        controller.land()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()