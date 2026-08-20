import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from rclpy.parameter import Parameter
import os
import math
import struct
import threading
import serial
from cobs import cobs
import crcmod.predefined

# Tipli, varsayilansiz parametreler. Degerlerin tek tanimi
# robotaksi_interface/config/robotaksi_interface.yaml; arac olculeri
# (wheelbase_m, track_width_m, max_steer_*) robotaksi_bringup'in
# vehicle_params.yaml'inda, cunku cmd_vel_mux ve joystick_teleop ayni sayilari
# okumak zorunda.
REQUIRED_PARAMETERS = {
    'serial_paths': Parameter.Type.STRING_ARRAY,
    'serial_fallback_paths': Parameter.Type.STRING_ARRAY,
    'baud_rate': Parameter.Type.INTEGER,
    'publish_tf': Parameter.Type.BOOL,
    'odom_frame': Parameter.Type.STRING,
    'base_frame': Parameter.Type.STRING,
    'wheelbase_m': Parameter.Type.DOUBLE,
    'track_width_m': Parameter.Type.DOUBLE,
    'max_steer_angle_deg': Parameter.Type.DOUBLE,
    'max_steer_speed_deg_s': Parameter.Type.DOUBLE,
    'pose_covariance_diagonal': Parameter.Type.DOUBLE_ARRAY,
    'twist_covariance_diagonal': Parameter.Type.DOUBLE_ARRAY,
}


def get_required_parameter(node, name):
    parameter = node.get_parameter(name)
    if parameter.type_ == Parameter.Type.NOT_SET:
        raise RuntimeError(
            f"Required ROS parameter '{name}' is missing. It is defined in "
            f"robotaksi_interface/config/robotaksi_interface.yaml or "
            f"robotaksi_bringup/config/vehicle_params.yaml; launch this node "
            f"through robotaksi_interface.launch.py.")
    return parameter.value


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

        # Parametreleri tanımla. Hepsi TIPLI ve VARSAYILANSIZ: degerlerin tek
        # tanimi robotaksi_interface/config/robotaksi_interface.yaml (ve arac
        # olculeri icin robotaksi_bringup/config/vehicle_params.yaml). Eksik
        # bir anahtar acilista parametre adiyla hata verir.
        #
        # Burada eskiden iki katmanli bir varsayilan zinciri vardi: kodda
        # yazili degerler (wheelbase 1.5, max_steer 35.0) ve arkasindan
        # vehicle_params.yaml'i ELLE okuyup "deger hala kodun varsayilanina
        # esitse" uzerine yazan bir blok. Bunun sonucu, YAML'da bir anahtar
        # yeniden adlandirildiginda ya da parametre kod varsayilaniyla ayni
        # sayiya ayarlandiginda dugumun sessizce yanlis olcuyle calismasiydi.
        for parameter_name, parameter_type in REQUIRED_PARAMETERS.items():
            self.declare_parameter(parameter_name, parameter_type)

        self.serial_paths = list(get_required_parameter(self, 'serial_paths'))
        self.baud = get_required_parameter(self, 'baud_rate')
        self.L = get_required_parameter(self, 'wheelbase_m')
        self.max_steer = get_required_parameter(self, 'max_steer_angle_deg')
        self.max_steer_speed = get_required_parameter(self, 'max_steer_speed_deg_s')
        self.publish_tf = get_required_parameter(self, 'publish_tf')
        self.odom_frame = get_required_parameter(self, 'odom_frame')
        self.base_frame = get_required_parameter(self, 'base_frame')
        self.pose_covariance_diagonal = list(
            get_required_parameter(self, 'pose_covariance_diagonal'))
        self.twist_covariance_diagonal = list(
            get_required_parameter(self, 'twist_covariance_diagonal'))

        # DOCKER HOTPLUG FIX: udev bazen /dev/serial/by-id/ eslemesini
        # konteyner icinde kuramiyor, o yuzden dogrudan cihaz yollari en sona
        # eklenir. Liste de config'ten geliyor.
        for fallback_path in get_required_parameter(self, 'serial_fallback_paths'):
            if fallback_path not in self.serial_paths:
                self.serial_paths.append(fallback_path)

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
                    t.header.frame_id = self.odom_frame
                    t.child_frame_id = self.base_frame
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation = q
                    self.tf_broadcaster.sendTransform(t)
                
                # Publish Odometry message
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = self.odom_frame
                odom.child_frame_id = self.base_frame
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = q
                odom.twist.twist.linear.x = speed
                odom.twist.twist.angular.z = omega
                # Covariance was left at all zeros. robot_localization reads a
                # zero variance as infinite confidence, so the EKF trusted this
                # dead-reckoned pose absolutely and became ill-conditioned.
                # Filled from parameters so it can be tuned on the vehicle.
                #
                # Pose is deliberately loose: x and y here are an unbounded
                # integration of wheel speed and steering angle, so their real
                # uncertainty grows with distance and no fixed number is
                # honest. The EKF is configured to fuse only twist.linear.x
                # from this message (see robotaksi_localization.yaml, odom1_config),
                # which is the
                # part wheel odometry actually measures; the pose values ride
                # along for anything else that subscribes.
                for i, var in enumerate(self.pose_covariance_diagonal):
                    odom.pose.covariance[i * 7] = var
                for i, var in enumerate(self.twist_covariance_diagonal):
                    odom.twist.covariance[i * 7] = var
                self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
