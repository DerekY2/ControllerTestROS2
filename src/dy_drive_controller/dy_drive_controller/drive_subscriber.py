from math import sqrt
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dy_drive_controller.constants import DriveConstants, JoystickConstants, Paths

def map_range(value, old_min, old_max, new_min, new_max, deadzone):
    if abs(value)<deadzone:
      return 0.0
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

class DriveSubscriber(Node):
  def __init__(self):
    super().__init__("drive_subscriber")
    self.twist = Twist()
    self.estop = Bool()
    self.estop.data = False
    self.estopTimeout = Bool()
    self.estopTimeout.data = False
    self.lastTimestamp = 0
    self.lastEstopRegistered = 0
    self.slewRateTimestamp = 0.0
    self.previous_speed = 0.0
    self.scalar = 1

    # modify axis range?
    self.declare_parameter("limiter", DriveConstants.limiter_active)
    self.limiter = (
        self.get_parameter("limiter").get_parameter_value().bool_value
    )
    # Limit max speed
    self.declare_parameter("limiter_max_speed", DriveConstants.limiter_max_speed)
    self.limiter_max_speed = (
        self.get_parameter("limiter_max_speed").get_parameter_value().double_value
    )
    # Limit max turn speed
    self.declare_parameter("limiter_max_turn", DriveConstants.limiter_max_turn)
    self.limiter_max_turn = (
        self.get_parameter("limiter_max_turn").get_parameter_value().double_value
    )
    # Max drive voltage
    self.declare_parameter("voltage_max_speed", DriveConstants.voltage_max_speed)
    self.voltage_max_speed = (
        self.get_parameter("voltage_max_speed").get_parameter_value().double_value
    )
    # Max turn voltage
    self.declare_parameter("voltage_max_turn", DriveConstants.voltage_max_turn)
    self.voltage_max_turn = (
        self.get_parameter("voltage_max_turn").get_parameter_value().double_value
    )

    # E-stop status publisher
    self.setEstop = self.create_publisher(Twist, Paths.estop, 1)

    # drive command publisher
    self.drive_publisher = self.create_publisher(Twist, Paths.cmd_vel, 1)

    self.active = True

    # Create joystick subscription
    self.drive_controller = self.create_subscription(Joy, Paths.speeds_commanded, self.joystick_container, 1)

    # self.slew_rate_timer = self.create_timer(0.1, self.update_slew_rate_timestamp())

    # def update_slew_rate_timestamp(self):
    #   self.slewRateTimestamp = Node.get_clock(self).now().seconds_nanoseconds()[0]

  
  # handle joystick timeout (i.e. disconnect)
  def joystick_timeout_handler(self):
    # has it been 2 secs since lastTimestamp privded from joystick feedback?
    if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2.0): # (currentTime - lastTime > 2)?
      self.estopTimeout.data = True
      self.setEstop.publish(self.estopTimeout)
    elif(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp <= 2.0 and (self.estopTimeout.data and not self.estop)):
      self.estopTimeout.data = False
      self.setEstop.publish(self.estopTimeout)
  
  def drive(self, xSpeed, rotation):
    self.twist.linear.x = sqrt(xSpeed) # drive
    self.twist.angular.z = sqrt(rotation) # turn
    self.drive_publisher.publish(self.twist)

  # def magRateLimiter(self, speed_commanded, max_change):
  #   current_time = Node.get_clock(self).now().seconds_nanoseconds()[0]
  #   self.previous_speed = self.previous_state.axes[JoystickConstants.xAxis]
  #   change = speed_commanded - self.previous_speed
  #   if change <max_change:
  #     return self.previous_speed+change
  #   else:

  #     pass
  #   if abs(change)>max_change:
  #     change = max_change if change > 0 else -max_change
  #   return self.previous_speed + change
  
  # # def rotRateLimiter(self, rotation, max_change)

  # teleop joystick controls
  def joystick_container(self, joystick: Joy):

    # toggle drivetrain control
    if joystick.buttons[JoystickConstants.toggleDriveButton] == 1 and self.previous_state.buttons[7] == 0:
      self.active = not self.active
    self.previous_state = joystick

    # e-stop
    if joystick.buttons[0] == 1 and ((Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastEstopRegistered) > JoystickConstants.estop_debounce_seconds):
      self.estop.data = not self.estop.data
      self.drive(self, 0.0, 0.0)
      self.setEstop.publish(self.estop)
    elif joystick.buttons[0] == 1 and Node.get_clock(self).now().seconds_nanoseconds()[0] < JoystickConstants.estop_debounce_seconds:
      self.estop.data = not self.estop.data
      self.drive(self, 0.0, 0.0)
      self.setEstop.publish(self.estop)

    # arcade drive
    if self.limiter and self.active:
      self.drive(
        map_range(joystick.axes[JoystickConstants.xAxis], JoystickConstants.axis_min, JoystickConstants.axis_max, -self.limiter_max_speed, self.limiter_max_speed, JoystickConstants.axis_deadzone),
        map_range(joystick.axes[JoystickConstants.zAxis], JoystickConstants.axis_min, JoystickConstants.axis_max, -self.limiter_max_turn, self.limiter_max_turn, JoystickConstants.axis_deadzone)
      )
    elif self.active:
      self.drive(
        map_range(joystick.axes[JoystickConstants.xAxis], JoystickConstants.axis_min, JoystickConstants.axis_max, self.voltage_max_speed, self.voltage_max_speed, JoystickConstants.axis_deadzone),
        map_range(joystick.axes[JoystickConstants.zAxis], JoystickConstants.axis_min, JoystickConstants.axis_max, self.voltage_max_turn, self.voltage_max_turn, JoystickConstants.axis_deadzone)
      )
    

    self.lastTimeStamp = joystick.header.stamp.sec


def main(args=None):
  rclpy.init(args=args)
  node = DriveSubscriber()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == '__main__':
  main()
