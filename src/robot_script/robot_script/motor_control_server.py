#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
from std_srvs.srv import SetBool

from threading import Thread
from time import sleep

class MotorController():
    def __init__(self):
        self.state = "READY"

    def startPushing(self):
        self.state = "ONGOING"
        thread = Thread(target = self.waitForChangeState, args = ("READY", 5, ))
        thread.start()
        

    def stopPushing(self):
        self.state = "STOPPING"
        thread = Thread(target = self.waitForChangeState, args = ("READY", 2, ))
        thread.start()

    def getMotorState(self):
        return self.state
    
    def waitForChangeState(self, state, second):
        sleep(second)
        self.state = state


class MotorControlServer(Node):

    def __init__(self):
        super().__init__("motor_control_server")

        self.motor = MotorController()
        
        self.server = self.create_service(SetBool, "motor_control", self.callback_motor_control_service)
        self.get_logger().info("motor_control_server has been started (motor_state topic and motor_control server)")

        self.publisher = self.create_publisher(String, "motor_state", 10)
        self.timer = self.create_timer(0.5, self.publish_motor_state)

    def callback_motor_control_service(self, request, response):
        
        self.get_logger().info("Receive: " + str(request.data))

        current_motor_state = self.motor.getMotorState()

        if request.data == True:

            if current_motor_state == "READY":

                try:
                    self.get_logger().info("Starting motor...")
                    self.motor.startPushing()

                except Exception as e:
                    self.get_logger().info("Error staring motor: " + str(e))
                    response.success = False
                    response.messsage = "Error staring motor: " + str(e)
                    return response
                
                self.get_logger().info("Initiate starting motor successful")
                response.success = True
                response.message = "OK"
            
            else:

                self.get_logger().info("Motor state is not READY (Currect state : " + current_motor_state + "). Abort")
            
                response.success = False
                response.message = "Motor is not ready (Current state : " + current_motor_state + ")"
               
            return response
        

        self.get_logger().info("Stopping motor...")
        try:
            self.motor.stopPushing()
        except Exception as e:
            self.get_logger().info("Error stopping motor: " + str(e))
            response.success = False
            response.messsage = "Error stopping motor: " + str(e)
            return response
        
        self.get_logger().info("Initiate stopping motor successful")
        response.success = True
        response.message = "OK"
        return response
    
    def publish_motor_state(self):
        msg = String()
        msg.data = self.motor.getMotorState()
        
        self.publisher.publish(msg)
        
        
    

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

