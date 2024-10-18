from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from constants import DriveConstants, JoystickConstants

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

class DriveController(Node):

  def __init__(self):
    super().__init__("drive_controller")
    self.twist = Twist()
    self.estop = Bool()
    self.estop.data = False
    self.estopTimeout = Bool()
    self.estopTimeout.data = False
    self.lastTimestamp = 0

    # limit axis range
    self.declare_parameter("PID", DriveConstants.limiter_active)
    self.pidMode = (
        self.get_parameter("PID").get_parameter_value().bool_value
    )
    # PID Max speed
    self.declare_parameter("PID_max_speed", DriveConstants.limiter_max_speed)
    self.MAX_PID_SPEED = (
        self.get_parameter("PID_max_speed").get_parameter_value().double_value
    )
    # PID max turn speed
    self.declare_parameter("PID_max_turn", DriveConstants.limiter_max_turn)
    self.MAX_PID_TURN = (
        self.get_parameter("PID_max_turn").get_parameter_value().double_value
    )
    # Max drive voltage
    self.declare_parameter("voltage_max_speed", DriveConstants.voltage_max_speed)
    self.MAX_VOLTAGE_SPEED = (
        self.get_parameter("voltage_max_speed").get_parameter_value().double_value
    )
    # Max turn voltage
    self.declare_parameter("voltage_max_turn", DriveConstants.voltage_max_turn)
    self.MAX_VOLTAGE_TURN = (
        self.get_parameter("voltage_max_turn").get_parameter_value().double_value
    )

    self.active = True

    # Drivespeed publisher (drive speed, turn speed)
    self.drive = self.create_publisher(Twist, DriveConstants.cmd_vel, 1)

    # E-stop status publisher
    self.setEstop = self.create_publisher(Twist, DriveConstants.estop, 1)
    self.setEstop.publish(self.estop) #init as not e-stopped

    # Create joystick subscription
    self.drive_controller = self.create_subscription(Joy, JoystickConstants.drive, None, 10)
    
    self.timer = self.create_timer(JoystickConstants.echo_frequency, self.joystick_timeout_handler)

  # handle joystick timeout (i.e. disconnect)
  def joystick_timeout_handler(self):
      # has it been 2 secs since lastTimestamp privded from joystick feedback?
      if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2): # (currentTime - lastTime > 2)?
          self.estopTimeout.data = True
          self.setEstop.publish(self.estopTimeout)
      elif(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp <= 2 and (self.estopTimeout.data and not self.estop)):
          self.estopTimeout.data = False
          self.setEstop.publish(self.estopTimeout) 

def main(args=None):
  rclpy.init(args=args)
  node = DriveController()
  rclpy.spin(node)
  rclpy.shutdown()



if __name__ == '__main__':
  main()
