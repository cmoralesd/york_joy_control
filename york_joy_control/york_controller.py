from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class MyRobotController(Node):

    def __init__(self):
        super().__init__('state_publisher')
        qos_profile = QoSProfile(depth=10)

        # inicializa variables
        self.time_last = self.get_clock().now()   # start time in nanoseconds
        self.odom = {'x':0.0, 'y':0.0, 'th':0.0}
        self.frw_joint_pos = 0
        self.flw_joint_pos = 0
        self.brw_joint_pos = 0
        self.blw_joint_pos = 0


         # initializa publicadores y suscriptores
        self.cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, qos_profile)

        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.get_logger().info(f"{self.get_name()} started")
        

    def cmd_vel_callback(self, msg):
        # lee el mensaje
        vx = msg.linear.x
        vy = msg.linear.y
        vr = msg.angular.z
        time_now = self.get_clock().now()

        # Aquí un controlador real debiera enviar órdenes a los motores
        # y recibir la posición y velocidad real desde los encoders del robot.
        # Por ahora, calculamos con los mismos datos de velocidad...

        vx_real = vx
        vy_real = vy
        vr_real = vr

        # actualiza la posición de las articulaciones y la odometría
        self.update_joint_positions(vx_real, vy_real, vr_real, time_now)
        self.update_odom(vx_real, vy_real, vr_real, time_now)
        x = self.odom['x']
        y = self.odom['y']
        th = self.odom['th']

        # prepara el mensaje de odometría para publicación
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.header.stamp = time_now.to_msg()
        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, th) # roll,pitch,yaw

        self.get_logger().info(f"x: {x}")
        self.get_logger().info(f"y: {y}")
        self.get_logger().info(f"th: {th}")

        # prepara el mensaje de posición de articulaciones, para publicación
        joint_state = JointState()
        joint_state.header.stamp = time_now.to_msg()
        joint_state.name = ['flw_joint', 'frw_joint', 'blw_joint', 'brw_joint']
        joint_state.position = [self.flw_joint_pos, self.frw_joint_pos, self.flw_joint_pos, self.frw_joint_pos]
        
        # y publica
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)


    def update_odom(self, vx, vy, vr, time_now):
        # calcula el tiempo transcurrido
        dt = (time_now.nanoseconds - self.time_last.nanoseconds) * 10**-9
        if dt > 0.1: dt = 0
        self.time_last = time_now
        # calcula desviaciones
        delta_th = vr*dt
        delta_x = (vx * cos(delta_th) - vy * sin(delta_th)) * dt
        delta_y = (vx * sin(delta_th) + vy * cos(delta_th)) * dt
        # actualiza odómetro
        self.odom['x'] += cos(self.odom['th']) * delta_x - sin(self.odom['th']) * delta_y
        self.odom['y'] += sin(self.odom['th']) * delta_x + cos(self.odom['th']) * delta_y
        self.odom['th'] += delta_th

    def update_joint_positions(self, vx, vy, vr, time_now):
        r1, r2 = 0.0384, 0.01 # radio mayor y menor de las ruedas mecanum
        a, b = 0.125, 0.105 # distancias entre ruedas
        # vx, vy, vr son los componentes de la velocidad del robot
        blw_joint_velocity = (-vr*(a*r2+b*(r1+r2))+r2*vy+vx*(r1+r2))/((r1)**2+2*r1*r2+2*r2**2)
        brw_joint_velocity = (vr*(a*r2+b*(r1+r2))-r2*vy+vx*(r1+r2))/((r1)**2+2*r1*r2+2*r2**2)
        frw_joint_velocity = (vr*(a*r2+b*(r1+r2))+r2*vy+vx*(r1+r2))/((r1)**2+2*r1*r2+2*r2**2)
        flw_joint_velocity = (-vr*(a*r2+b*(r1+r2))-r2*vy+vx*(r1+r2))/((r1)**2+2*r1*r2+2*r2**2)

        # actualiza las posiciones de las articulaciones
        dt = (time_now.nanoseconds - self.time_last.nanoseconds) * 10**-9
        self.blw_joint_pos += blw_joint_velocity * dt
        self.brw_joint_pos += brw_joint_velocity * dt
        self.flw_joint_pos += frw_joint_velocity * dt
        self.frw_joint_pos += flw_joint_velocity * dt


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    try:
        rclpy.init(args=args)
        my_robot_controller = MyRobotController()
        rclpy.spin(my_robot_controller)
    except KeyboardInterrupt: 
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()