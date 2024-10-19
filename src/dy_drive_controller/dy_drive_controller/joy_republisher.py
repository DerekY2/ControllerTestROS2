import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dy_drive_controller.constants import JoystickConstants, Paths

def map_range(value, old_min, old_max, new_min, new_max, deadzone):
    if abs(value)<deadzone:
      return 0.0
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

class JoyRepublisher(Node):

  def __init__(self):
    super().__init__("joy_republisher")
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


    # Create joystick subscription
    self.drive_controller = self.create_subscription(Joy, Paths.joy, self.publish_data, 10)  

    self.joystick_echo_timer = self.create_timer(JoystickConstants.echo_frequency, self.joystick_timeout_handler)

    # Joystick publisher
    self.joystick_publisher = self.create_publisher(Joy, Paths.speeds_commanded, 1)

    # E-stop status publisher
    self.setEstop = self.create_publisher(Bool, Paths.estop, 1)
    self.setEstop.publish(self.estop) #init as not e-stopped


  # def update_slew_rate_timestamp(self):
  #   self.slewRateTimestamp = Node.get_clock(self).now().seconds_nanoseconds()[0]

  def publish_data(self, msg: Joy):
    self.joystick_publisher.publish(msg)

  # handle joystick timeout (i.e. disconnect)
  def joystick_timeout_handler(self):
    # has it been 2 secs since lastTimestamp privded from joystick feedback?
    if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2.0): # (currentTime - lastTime > 2)?
      self.estopTimeout.data = True
      self.setEstop.publish(self.estopTimeout)
    elif(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp <= 2.0 and (self.estopTimeout.data and not self.estop)):
      self.estopTimeout.data = False
      self.setEstop.publish(self.estopTimeout)
  
def main(args=None):
  rclpy.init(args=args)
  node = JoyRepublisher()
  rclpy.spin(node)
  rclpy.shutdown()



if __name__ == '__main__':
  main()
