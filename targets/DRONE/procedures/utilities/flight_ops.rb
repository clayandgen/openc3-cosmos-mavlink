# ---------------------------------------------------------------------------
# Common flight operation sequences for PX4 drone missions
#
# Requires cmd_and_ack to be loaded first.
#
# PX4 DO_SET_MODE via COMMAND_LONG uses separate params:
#   MODE_1 = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
#   CUSTOM_MODE_2 = main_mode (4 = AUTO)
#   CUSTOM_SUBMODE_3 = sub_mode  (3 = LOITER, 4 = MISSION, 5 = RTL)
# base_mode bit 7 (0x80 / 128) = armed flag
# ---------------------------------------------------------------------------

# Wait for a heartbeat from the vehicle
def wait_for_heartbeat(timeout: 30)
  puts "--- Waiting for vehicle heartbeat ---"
  wait_check("DRONE HEARTBEAT SYSTEM_STATUS == 'STANDBY'", timeout)
  puts "Heartbeat received. Vehicle is in STANDBY."
end

# Run preflight / pre-arm checks
def run_preflight_checks(timeout: 10)
  puts "--- Running pre-arm checks ---"
  cmd_and_ack("RUN_PREARM_CHECKS", timeout: timeout)
  puts "Pre-arm checks passed."
end

# Set AUTO LOITER mode (safe hold mode before arming)
def set_loiter_mode
  puts "--- Setting AUTO LOITER mode ---"
  cmd_and_ack("DO_SET_MODE", MODE_1: 1, CUSTOM_MODE_2: 4, CUSTOM_SUBMODE_3: 3)
  wait_check("DRONE HEARTBEAT CUSTOM_MODE == 50593792", 5)
  puts "Vehicle is in AUTO LOITER mode."
end

# Arm the vehicle (prompts operator for confirmation)
def arm_vehicle(confirm_message: "Confirm: Area is clear and safe to arm?")
  puts "--- Arming vehicle ---"
  prompt(confirm_message)
  cmd_and_ack("COMPONENT_ARM_DISARM", ARM_1: 1, FORCE_2: 0)
  wait_check("DRONE HEARTBEAT BASE_MODE >= 128", 5)
  puts "Vehicle is ARMED."
end

# Switch to AUTO MISSION mode to begin mission execution
def start_mission_mode
  puts "--- Starting mission ---"
  cmd_and_ack("DO_SET_MODE", MODE_1: 1, CUSTOM_MODE_2: 4, CUSTOM_SUBMODE_3: 4)
  wait_check("DRONE HEARTBEAT CUSTOM_MODE == 67371008", 5)
  puts "Vehicle is in AUTO MISSION mode. Mission is executing."
end

# Wait for the vehicle to land (altitude drops below threshold)
def wait_for_landing(timeout: 120)
  puts "--- Waiting for landing ---"
  wait_check("DRONE GLOBAL_POSITION_INT RELATIVE_ALT < 1000", timeout)
  puts "Vehicle has landed."
end

# Disarm the vehicle
def disarm_vehicle(settle_time: 3)
  puts "--- Disarming vehicle ---"
  wait(settle_time)
  cmd_and_ack("COMPONENT_ARM_DISARM", ARM_1: 0, FORCE_2: 0, timeout: 10)
  wait_check("DRONE HEARTBEAT BASE_MODE < 128", 5)
  puts "Vehicle is DISARMED."
end
