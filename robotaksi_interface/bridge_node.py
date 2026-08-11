import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import math
import struct
import threading
import serial
from cobs import cobs
import crcmod.predefined

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = Quaternion()
    q.x = cj*sc - sj*cs
    q.y = cj*ss + sj*cc
    q.z = cj*cs - sj*sc
    q.w = cj*cc + sj*ss
    return q

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')

        # Parametreleri tanımla
        self.declare_parameter('serial_paths', ['/dev/ttyACM0'])
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('wheelbase_m', 1.5)
        self.declare_parameter('track_width_m', 1.2)
        self.declare_parameter('max_steer_angle_deg', 35.0)
        self.declare_parameter('max_steer_speed_deg_s', 20.0)
        self.declare_parameter('publish_tf', False)

        self.serial_paths = self.get_parameter('serial_paths').value
        self.baud = self.get_parameter('baud_rate').value
        self.L = self.get_parameter('wheelbase_m').value
        self.max_steer = self.get_parameter('max_steer_angle_deg').value
        self.max_steer_speed = self.get_parameter('max_steer_speed_deg_s').value
        self.publish_tf = self.get_parameter('publish_tf').value

        try:
            pkg_share = get_package_share_directory('robotaksi_interface')
            config_path = os.path.join(pkg_share, 'config', 'vehicle_params.yaml')
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    bridge_params = config.get('bridge_node', {}).get('ros__parameters', {})
                    if 'serial_paths' in bridge_params and self.serial_paths == ['/dev/ttyACM0']:
                        self.serial_paths = bridge_params['serial_paths']
                    if 'baud_rate' in bridge_params and self.baud == 1000000:
                        self.baud = bridge_params['baud_rate']
                    if 'wheelbase_m' in bridge_params and self.L == 1.5:
                        self.L = bridge_params['wheelbase_m']
                    if 'max_steer_angle_deg' in bridge_params and self.max_steer == 35.0:
                        self.max_steer = bridge_params['max_steer_angle_deg']
                    if 'max_steer_speed_deg_s' in bridge_params and self.max_steer_speed == 20.0:
                        self.max_steer_speed = bridge_params['max_steer_speed_deg_s']
                    if 'publish_tf' in bridge_params:
                        self.publish_tf = bridge_params['publish_tf']
        except Exception as e:
            self.get_logger().warn(f"Could not load fallback config: {e}")

        if isinstance(self.serial_paths, str):
            self.serial_paths = [self.serial_paths]
            
        # DOCKER HOTPLUG FIX: udev sometimes fails to map /dev/serial/by-id/ inside the container.
        # We append direct device paths as ultimate fallbacks.
        fallback_paths = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for fp in fallback_paths:
            if fp not in self.serial_paths:
                self.serial_paths.append(fp)

        self.crc_maxim = crcmod.predefined.mkCrcFun('crc-8-maxim')
        self.board_ready = False
        
        # State variables for Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_steer_rad = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Serial port kurulumu (Bağlı olan ilk cihazı bul)
        self.ser = None
        for path in self.serial_paths:
            if not os.path.exists(path):
                continue
            try:
                self.ser = serial.Serial(path, self.baud, timeout=0.1)
                self.ser.reset_input_buffer()
                self.get_logger().info(f"Successfully connected to Arduino at {path} ({self.baud} baud). Waiting for board to boot...")
                break
            except Exception as e:
                self.get_logger().warn(f"Port unavailable {path}: {e}")
        
        if not self.ser:
            self.get_logger().error("COULD NOT CONNECT TO ANY SPECIFIED ARDUINO PORT! Check connections or permissions.")

        # Twist Subscriber (Teleop'dan gelen mesajlar)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Aux Subscriber (Teleop'dan gelen Auxiliary mesajları)
        self.aux_subscription = self.create_subscription(
            String,
            'aux_cmd',
            self.aux_cmd_callback,
            10)

        # Thread for reading telemetry
        if self.ser:
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

    def send_command(self, cmd_id, payload=b""):
        if not self.ser or not self.board_ready: return
        buf = bytearray([cmd_id]) + payload
        c = self.crc_maxim(bytes(buf))
        buf.append(c)
        encoded = cobs.encode(bytes(buf))
        self.ser.write(encoded + b'\x00')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        # Ackermann Dönüşümü: delta = arctan(L * omega / v)
        if abs(v) > 0.01:
            steer_rad = math.atan((self.L * omega) / v)
        else:
            # Duruyorken yerinde dönme komutu gelirse (Ackermann araçlar yerinde dönemez,
            # ancak direksiyonu çevirebiliriz)
            steer_rad = math.copysign(math.radians(self.max_steer), omega) if omega != 0 else 0.0

        self.current_steer_rad = steer_rad

        # DİKKAT (İşaret Uyumsuzluğu):
        # ROS 2 standartlarında Pozitif (+) yön Sola dönüş demektir (Counter-Clockwise).
        # Ancak Arduino SteerController.cpp'de Pozitif (+) açı Sağa dönüş olarak tasarlanmıştır.
        # Bu uyumsuzluğu donanımı ellememek adına Python tarafında -1 ile çarparak (tersleyerek) çözüyoruz.
        steer_deg = -math.degrees(steer_rad)
        steer_deg = max(-self.max_steer, min(self.max_steer, steer_deg))

        self.get_logger().info(f"Twist: v={v:.2f}, w={omega:.2f} -> Steer={steer_deg:.1f} deg")

        # 0x20 CMD_VELOCITY_TARGET (Float m/s)
        payload_vel = struct.pack("<f", v)
        self.send_command(0x20, payload_vel)

        # 0x10 CMD_STEER_TARGET (int8_t angle, float speed_deg_s)
        steer_int = int(steer_deg)
        steer_speed_deg_s = float(self.max_steer_speed)
        payload_steer = struct.pack("<bf", steer_int, steer_speed_deg_s)
        self.send_command(0x10, payload_steer)

    def aux_cmd_callback(self, msg):
        parts = msg.data.split(" | ")
        horn = False
        headlight = False
        signal = 0 # 0: OFF, 1: RIGHT, 2: LEFT, 3: HAZARD
        
        for p in parts:
            if p.startswith("HORN:"):
                horn = (p.split(":")[1] == "ON")
            elif p.startswith("HEADLIGHT:"):
                headlight = (p.split(":")[1] == "ON")
            elif p.startswith("BLINKER:"):
                val = p.split(":")[1]
                if val == "RIGHT": signal = 1
                elif val == "LEFT": signal = 2
                elif val == "HAZARD": signal = 3
                else: signal = 0
                
        # 0x30 CMD_AUX_CONTROL (bool headlight, bool horn, uint8 signals)
        payload_aux = struct.pack('<??B', headlight, horn, signal)
        self.send_command(0x30, payload_aux)

    def serial_read_loop(self):
        buffer = bytearray()
        while rclpy.ok():
            if self.ser:
                try:
                    to_read = max(1, self.ser.in_waiting)
                    chunk = self.ser.read(to_read)
                    for b in chunk:
                        if b == 0x00:
                            if len(buffer) > 0:
                                try:
                                    decoded = cobs.decode(bytes(buffer))
                                    self.process_packet(decoded)
                                except Exception:
                                    pass
                            buffer.clear()
                        else:
                            buffer.append(b)
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial read error: {e}")
                    break

    def process_packet(self, decoded):
        if len(decoded) < 2: return
        
        expected_crc = self.crc_maxim(decoded[:-1])
        if expected_crc != decoded[-1]: return

        cmd_id = decoded[0]
        if cmd_id == 0x70 and len(decoded) == 19: # RES_TELEMETRY (cmd:1, speed:4, pulses:4, t_speed:4, t_percent:4, c_angle:1, crc:1)
            
            if not self.board_ready:
                self.board_ready = True
                self.get_logger().info("Board telemetry received! Arduino is fully booted and ready to transmit.")

            payload = decoded[1:-1]
            speed, total_pulses, t_speed, t_percent, current_angle = struct.unpack('<fIffb', payload)
            self.get_logger().info(f"Telemetry: Speed={speed:.2f} m/s, TgtSpd={t_speed:.2f} m/s, Steer={current_angle} deg")
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now

            if dt > 0 and dt < 1.0:
                # Forward kinematics using Bicycle model
                # omega = v * tan(delta) / L
                omega = speed * math.tan(self.current_steer_rad) / self.L
                
                self.theta += omega * dt
                self.x += speed * math.cos(self.theta) * dt
                self.y += speed * math.sin(self.theta) * dt
                
                # Create quaternion from theta (yaw)
                q = quaternion_from_euler(0, 0, self.theta)
                
                if self.publish_tf:
                    # Broadcast TF only when explicitly enabled. Localization owns odom->base.
                    t = TransformStamped()
                    t.header.stamp = now.to_msg()
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation = q
                    self.tf_broadcaster.sendTransform(t)
                
                # Publish Odometry message
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = q
                odom.twist.twist.linear.x = speed
                odom.twist.twist.angular.z = omega
                self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
