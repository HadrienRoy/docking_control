#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import pi

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from map_simulator.srv import Spawn


class Vel2Joints(Node):    

    def __init__(self):        
        super().__init__('vel2joints')
        
        self.js = JointState()
        # self.js.name = ["wheel", "torso", "neck"]
        self.js.name = ["base_joint", "wheel_left_joint", "wheel_right_joint"]
        self.js.position = [0.,0.,0.]
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.cmd_sub  # prevent unused variable warning
        self.v = 0
        self.w = 0    
        
        self.dt = self.declare_parameter("dt", 0.1).value
        self.timer = self.create_timer(self.dt, self.publish)
                
        self.spawn(self.declare_parameter("static_tf", False).value,
                   self.declare_parameter("x_pose", 0.0).value,
                   self.declare_parameter("y_pose", 0.0).value,
                   self.declare_parameter("yaw", 0.0).value
        )

        
    def spawn(self, static_tf, x_pose, y_pose, yaw):
        
        robot = self.get_namespace().strip('/')
        
        self.radius = .22
        
        # spawn robot in simulation        
        spawner = self.create_client(Spawn, '/simulator/spawn')
        while not spawner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')
            
        req = Spawn.Request()
        req.robot_namespace = self.get_namespace()
        
        req.radius = self.radius
        req.shape = req.SHAPE_CIRCLE
        req.static_tf_odom = static_tf
        
        req.x = x_pose
        req.y = y_pose
        req.theta = yaw
        req.laser_color = [0,255,0]
            
        self.future = spawner.call_async(req)
        self.get_logger().info(f'spawned robot {robot} in map_simulator')
                
    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

        
    def publish(self):
        self.js.position[0] += self.v*self.dt/self.radius
        self.js.position[1] = self.v*pi/12
        self.js.position[2] = self.w*pi/12
        self.js.header.stamp = self.get_clock().now().to_msg()        
        self.js_pub.publish(self.js)        
    

def main(args=None):
    
    rclpy.init(args=args)

    move_joints = Vel2Joints()

    rclpy.spin(move_joints)

    move_joints.destroy_timer(move_joints.timer)
    move_joints.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
