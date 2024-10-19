import math
from dataclasses import dataclass
from typing import List, Tuple

##############################################
@dataclass
class DriveConstants:

  voltage_max_speed = 1.0
  voltage_max_turn = 1.0

  limiter_active = False
  limiter_max_speed = 1.0
  limiter_max_turn = 1.0


##############################################
@dataclass
class JoystickConstants:

  xAxis = 1
  yAxis = 0
  zAxis = 3
  toggleDriveButton = 7 # menu - center.right


  axis_deadzone = 0.02
  axis_min = -1
  axis_max = 1

  magnitude_roc_limit = 0.1

  # feedback checks per second
  echo_frequency = 10 

  estop_debounce_seconds = 3.0


##############################################
@dataclass
class Paths:
  # drive topic paths
  cmd_vel = "/drive/cmd_vel"
  estop = "/drive/estop"

  # joystick topic paths
  joy = "/joy"
  drive = "/joystick/drive"
  speeds_commanded = "/drive/speeds_commanded"