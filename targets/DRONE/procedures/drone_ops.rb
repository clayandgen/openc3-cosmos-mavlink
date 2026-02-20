# ============================================================================
# Drone Operations Procedure (Mission-Based)
#
# A mission-based flight sequence for a PX4 drone in Gazebo sim.
# Sequence: connect -> upload mission -> preflight -> arm ->
#           switch to mission mode -> fly mission -> land -> disarm
#
# NAV_TAKEOFF via COMMAND_LONG does not work reliably on PX4.
# Instead, we upload a mission with a NAV_TAKEOFF item and switch
# to AUTO MISSION mode, which PX4 executes correctly.
# ============================================================================

load_utility '<%= target_name %>/procedures/utilities/cmd_and_ack.rb'
load_utility '<%= target_name %>/procedures/utilities/upload_mission.rb'
load_utility '<%= target_name %>/procedures/utilities/flight_ops.rb'

# ============================================================================
# 1. VERIFY CONNECTION
# ============================================================================
wait_for_heartbeat

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
# 3-6. PREFLIGHT, LOITER, ARM, START MISSION
# ============================================================================
run_preflight_checks
set_loiter_mode
arm_vehicle
start_mission_mode

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
wait_for_landing

# ============================================================================
# 8. DISARM
# ============================================================================
disarm_vehicle

puts "=== Drone operation complete ==="
