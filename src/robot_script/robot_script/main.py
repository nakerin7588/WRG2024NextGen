#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

from std_srvs.srv import SetBool
from example_interfaces.msg import String

from threading import Thread

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

class MotorStateSubscriber(Node):
    def __init__(self):
        super().__init__("motor_control_client")
        self.motor_state = "NULL"
        self.subscriber = self.create_subscription(String, "motor_state", self.callback_motor_state, 10)

    def callback_motor_state(self, msg):
        self.motor_state = msg.data
        # print("MSG : " + msg.data)

    def getMotorState(self):
        return self.motor_state

class MotorControlClient(Node):
    def __init__(self):
        super().__init__("motor_control_client")
        self.motor_state = "NULL"
        self.subscriber = self.create_subscription(String, "motor_state", self.callback_motor_state, 10)

    def call_push_service(self, startBool):
        client = self.create_client(SetBool, "motor_control")

        while not client.wait_for_service(1.0):
            self.get_logger().info("Waiting for motor_control service ...")

        request = SetBool.Request()
        request.data = startBool

        self.get_logger().info("Sending request to motor_control, data : " + str(startBool))

        future = client.call_async(request)
        future.add_done_callback(self.callback_motor_control)

    def callback_motor_control(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info("Service call successful, response: " + response.message)
            else:
                self.get_logger().info("Receive error from service, response: " + response.message)

        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))

    def callback_motor_state(self, msg):
        self.motor_state = msg.data
        print("MSG : " + msg.data)

    def getMotorState(self):
        return self.motor_state

def goToPoseAction(nav, motor_control_client, motor_state_subscriber, pose):

    nav.goToPose(pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)
    
    motor_control_client.call_push_service(True)
    print("Waiting for motor...")
    i = 0
    while motor_state_subscriber.getMotorState() == "READY":
        pass

    while motor_state_subscriber.getMotorState() != "READY":
        # i += 1
        # if i%10000 == 0:
        #     print("Current state : " + motor_state_subscriber.getMotorState())
        if motor_state_subscriber.getMotorState() == "ERROR":
            raise Exception("Error on motor")

    print("Pose push succeessful")
        
         
    

def main():

    #Init
    rclpy.init()
    nav = BasicNavigator()
    motor_control_client = MotorControlClient()
    motor_state_subscriber = MotorStateSubscriber()

    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)

    goal_pose = [
        create_pose_stamped(nav, 2.5, 1.0, 1.57),
        create_pose_stamped(nav, 2.0, 2.5, 3.14),
        create_pose_stamped(nav, 0.5, 1.0, -1.57)
    ]

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(motor_state_subscriber)

    et = Thread(target=executor.spin)
    et.start()

    # Set Initial Pose
    nav.setInitialPose(initial_pose)

    #wait for nav2
    nav.waitUntilNav2Active()
    

    goToPoseAction(nav, motor_control_client, motor_state_subscriber, goal_pose[0])
    goToPoseAction(nav, motor_control_client, motor_state_subscriber, goal_pose[1])
    goToPoseAction(nav, motor_control_client, motor_state_subscriber, goal_pose[2])

    #Send Result
    print(nav.getResult())

    et.join()
    #Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()