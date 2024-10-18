import math
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class DriveConstants:

  voltage_max_speed = 1.0
  voltage_max_turn = 1.0

  limiter_active = False
  limiter_max_speed = 1.0
  limiter_max_turn = 1.0

  # drive paths
  cmd_vel = "/drive/cmd_vel"
  estop = "/drive/estop"

class JoystickConstants:
  
  # feedback checks per second
  echo_frequency = 10 

  # joystick paths
  drive = "/joystick/drive"