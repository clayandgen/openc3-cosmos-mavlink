# ---------------------------------------------------------------------------
# Flashlight payload helpers
#
# Uses DO_SET_SERVO (MAV_CMD 183) to control the flashlight.
#   INSTANCE_1 = servo output number
#   PWM_2 = PWM value in microseconds
#
# Port 7: Deploy/stow servo
# Port 8: Flashlight on/off (turns on at ~1700 PWM)
# ---------------------------------------------------------------------------

FLASHLIGHT_SERVO_PORT = 7
FLASHLIGHT_STOW_PWM   = 425
FLASHLIGHT_DEPLOY_PWM = 1200

FLASHLIGHT_POWER_PORT = 8
FLASHLIGHT_OFF_PWM    = 1100
FLASHLIGHT_ON_PWM     = 1900

def flashlight_stow
  puts "--- Stowing flashlight ---"
  cmd("DRONE DO_SET_SERVO with INSTANCE_1 #{FLASHLIGHT_SERVO_PORT}, PWM_2 #{FLASHLIGHT_STOW_PWM}")
  puts "Flashlight stowed (PWM #{FLASHLIGHT_STOW_PWM})."
end

def flashlight_deploy
  puts "--- Deploying flashlight ---"
  cmd("DRONE DO_SET_SERVO with INSTANCE_1 #{FLASHLIGHT_SERVO_PORT}, PWM_2 #{FLASHLIGHT_DEPLOY_PWM}")
  puts "Flashlight deployed (PWM #{FLASHLIGHT_DEPLOY_PWM})."
end

def flashlight_on
  puts "--- Turning flashlight ON ---"
  cmd("DRONE DO_SET_SERVO with INSTANCE_1 #{FLASHLIGHT_POWER_PORT}, PWM_2 #{FLASHLIGHT_ON_PWM}")
  puts "Flashlight ON (PWM #{FLASHLIGHT_ON_PWM})."
end

def flashlight_off
  puts "--- Turning flashlight OFF ---"
  cmd("DRONE DO_SET_SERVO with INSTANCE_1 #{FLASHLIGHT_POWER_PORT}, PWM_2 #{FLASHLIGHT_OFF_PWM}")
  puts "Flashlight OFF (PWM #{FLASHLIGHT_OFF_PWM})."
end
