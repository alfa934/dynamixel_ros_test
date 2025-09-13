import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pygame
import time

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ps4_controller', 10)
        
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected!")
            return
            
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Initialized controller: {self.joystick.get_name()}")
        
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz

    def timer_callback(self):
        pygame.event.pump()
        
        msg = Float32MultiArray()
        
        for i in range(6):
            axis_value = self.joystick.get_axis(i)
            if axis_value > 0.9999:
                axis_value = 1.0
            elif axis_value < -0.9999:
                axis_value = -1.0
            else:
                axis_value = round(axis_value, 4)
                
            msg.data.append(axis_value)
        
        for i in range(11):
            button_value = float(self.joystick.get_button(i))
            msg.data.append(button_value)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()