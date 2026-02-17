# ============================================================================
# Drone Operations Procedure (Mission-Based)
#
# A mission-based flight sequence for a PX4 drone in Gazebo sim.
# Sequence: connect -> upload mission -> preflight -> arm ->
#           switch to mission mode -> fly mission -> land -> disarm
#
# PX4 DO_SET_MODE via COMMAND_LONG uses separate params:
#   PARAM1 = 1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
#   PARAM2 = main_mode (4 = AUTO)
#   PARAM3 = sub_mode  (3 = LOITER, 4 = MISSION, 5 = RTL)
# base_mode bit 7 (0x80 / 128) = armed flag
#
# NAV_TAKEOFF via COMMAND_LONG does not work reliably on PX4.
# Instead, we upload a mission with a NAV_TAKEOFF item and switch
# to AUTO MISSION mode, which PX4 executes correctly.
# ============================================================================

# ---------------------------------------------------------------------------
# Helper: send a COMMAND_LONG and verify the ACK comes back accepted
# ---------------------------------------------------------------------------
def cmd_and_ack(cmd_name, timeout: 5, **params)
  initial_count = tlm("DRONE COMMAND_ACK RECEIVED_COUNT")

  if params.empty?
    cmd("DRONE #{cmd_name}")
  else
    cmd_str = "DRONE #{cmd_name} with " + params.map { |k, v| "#{k} #{v}" }.join(", ")
    cmd(cmd_str)
  end

  wait_check("DRONE COMMAND_ACK RECEIVED_COUNT > #{initial_count}", timeout)

  result = tlm("DRONE COMMAND_ACK RESULT")
  if result != "MAV_RESULT_ACCEPTED"
    raise "Command #{cmd_name} failed with result: #{result}"
  end
end

# ---------------------------------------------------------------------------
# Helper: upload a mission to the vehicle using the MAVLink mission protocol
#
# Flow: MISSION_CLEAR_ALL -> MISSION_COUNT -> respond to MISSION_REQUEST_INT
#       for each item -> wait for MISSION_ACK
# ---------------------------------------------------------------------------
def upload_mission(items)
  puts "  Clearing existing mission..."
  ack_count = tlm("DRONE MISSION_ACK RECEIVED_COUNT")
  cmd("DRONE MISSION_CLEAR_ALL")
  wait_check("DRONE MISSION_ACK RECEIVED_COUNT > #{ack_count}", 5)

  clear_result = tlm("DRONE MISSION_ACK TYPE")
  raise "Clear mission failed: #{clear_result}" unless clear_result == "MAV_MISSION_ACCEPTED"
  puts "  Existing mission cleared."

  puts "  Uploading #{items.length} mission items..."
  req_count = tlm("DRONE MISSION_REQUEST_INT RECEIVED_COUNT")
  ack_count = tlm("DRONE MISSION_ACK RECEIVED_COUNT")

  cmd("DRONE MISSION_COUNT with COUNT #{items.length}")

  items.each_with_index do |item, i|
    # Wait for vehicle to request this sequence number
    wait_check("DRONE MISSION_REQUEST_INT RECEIVED_COUNT > #{req_count + i}", 5)
    requested_seq = tlm("DRONE MISSION_REQUEST_INT SEQ").to_i
    puts "    Vehicle requested item #{requested_seq}"

    # Send the requested mission item
    cmd("DRONE MISSION_ITEM_INT with " \
        "SEQ #{item[:seq]}, FRAME #{item[:frame]}, COMMAND #{item[:command]}, " \
        "CURRENT #{item[:current]}, AUTOCONTINUE #{item[:autocontinue]}, " \
        "PARAM1 #{item[:param1]}, PARAM2 #{item[:param2]}, " \
        "PARAM3 #{item[:param3]}, PARAM4 #{item[:param4]}, " \
        "X #{item[:x]}, Y #{item[:y]}, Z #{item[:z]}")
  end

  # Wait for final mission ACK
  wait_check("DRONE MISSION_ACK RECEIVED_COUNT > #{ack_count}", 10)
  upload_result = tlm("DRONE MISSION_ACK TYPE")
  raise "Mission upload failed: #{upload_result}" unless upload_result == "MAV_MISSION_ACCEPTED"
  puts "  Mission upload complete!"
end

# ============================================================================
# 1. VERIFY CONNECTION - wait for a heartbeat from the vehicle
# ============================================================================
puts "--- Waiting for vehicle heartbeat ---"
wait_check("DRONE HEARTBEAT SYSTEM_STATUS == 'MAV_STATE_STANDBY'", 30)
puts "Heartbeat received. Vehicle is in STANDBY."

# ============================================================================
# 2. BUILD AND UPLOAD MISSION
#    Read current GPS position, build a mission with:
#      Item 0: NAV_TAKEOFF to 10m
#      Item 1: NAV_WAYPOINT ~50m north at 10m
#      Item 2: NAV_RETURN_TO_LAUNCH
#    PX4 sets home position automatically on arm.
# ============================================================================
puts "--- Building mission ---"

# Read current position (raw degE7 integers from MAVLink)
home_lat_e7 = tlm("DRONE GLOBAL_POSITION_INT LAT").to_i
home_lon_e7 = tlm("DRONE GLOBAL_POSITION_INT LON").to_i

# Waypoint: ~50m north (approx 0.00045 degrees = 4500 degE7)
wp1_lat_e7 = home_lat_e7 + 4500
wp1_lon_e7 = home_lon_e7

puts "  Current position: lat=#{home_lat_e7 / 10000000.0}, lon=#{home_lon_e7 / 10000000.0}"
puts "  Waypoint 1: lat=#{wp1_lat_e7 / 10000000.0}, lon=#{wp1_lon_e7 / 10000000.0}, alt=10m"

# Frame 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT (alt relative to home)
# Frame 2 = MAV_FRAME_MISSION (for non-position commands)
mission_items = [
  # seq 0: NAV_TAKEOFF (cmd 22) to 10m above home
  { seq: 0, frame: 3, command: 22, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: home_lat_e7, y: home_lon_e7, z: 10.0 },
  # seq 1: NAV_WAYPOINT (cmd 16) ~50m north at 10m
  { seq: 1, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: wp1_lat_e7, y: wp1_lon_e7, z: 10.0 },
  # seq 2: NAV_RETURN_TO_LAUNCH (cmd 20) - frame 2 (MISSION) for non-position command
  { seq: 2, frame: 2, command: 20, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: 0, y: 0, z: 0 }
]

puts "--- Uploading mission ---"
upload_mission(mission_items)

# ============================================================================
# 3. PREFLIGHT CHECKS
# ============================================================================
puts "--- Running pre-arm checks ---"
cmd_and_ack("RUN_PREARM_CHECKS", timeout: 10)
puts "Pre-arm checks passed."

# ============================================================================
# 4. SET MODE TO AUTO LOITER (safe hold mode before arming)
#    PX4: PARAM1=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
#    PARAM2=4 (main_mode AUTO), PARAM3=3 (sub_mode LOITER)
# ============================================================================
puts "--- Setting AUTO LOITER mode ---"
cmd_and_ack("DO_SET_MODE", PARAM1: 1, PARAM2: 4, PARAM3: 3)
wait_check("DRONE HEARTBEAT CUSTOM_MODE == 50593792", 5)
puts "Vehicle is in AUTO LOITER mode."

# ============================================================================
# 5. ARM THE VEHICLE
#    PARAM1=1 to arm, PARAM2=0 (respect safety checks)
# ============================================================================
puts "--- Arming vehicle ---"
prompt("Confirm: Area is clear and safe to arm?")

cmd_and_ack("COMPONENT_ARM_DISARM", PARAM1: 1, PARAM2: 0)

# Verify armed via base_mode bit 7 (value >= 128 means armed)
wait_check("DRONE HEARTBEAT BASE_MODE >= 128", 5)
puts "Vehicle is ARMED."

# ============================================================================
# 6. SWITCH TO AUTO MISSION MODE - vehicle begins executing the mission
#    PX4 AUTO MISSION: PARAM2=4 (AUTO), PARAM3=4 (MISSION)
# ============================================================================
puts "--- Starting mission ---"
cmd_and_ack("DO_SET_MODE", PARAM1: 1, PARAM2: 4, PARAM3: 4)
wait_check("DRONE HEARTBEAT CUSTOM_MODE == 67371008", 5)
puts "Vehicle is in AUTO MISSION mode. Mission is executing."

# ============================================================================
# 7. MONITOR MISSION EXECUTION
# ============================================================================

# Wait for takeoff - vehicle should climb to ~10m (relative_alt is in mm)
puts "--- Waiting for takeoff to 10m ---"
wait_check("DRONE GLOBAL_POSITION_INT RELATIVE_ALT > 9000", 30)
puts "Reached target altitude."

# Wait for waypoint reached (item 1 is the waypoint)
puts "--- Flying to waypoint ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 1", 60)
puts "Waypoint reached."

# Wait for RTL to complete - altitude drops near ground (< 1m = 1000mm)
puts "--- Returning to launch ---"
wait_check("DRONE GLOBAL_POSITION_INT RELATIVE_ALT < 1000", 120)
puts "Vehicle has landed."

# ============================================================================
# 8. DISARM
# ============================================================================
puts "--- Disarming vehicle ---"
wait(3) # Let it settle on ground

cmd_and_ack("COMPONENT_ARM_DISARM", PARAM1: 0, PARAM2: 0, timeout: 10)
wait_check("DRONE HEARTBEAT BASE_MODE < 128", 5)
puts "Vehicle is DISARMED."

puts "=== Drone operation complete ==="
