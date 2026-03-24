# ============================================================================
# Rover Mission Procedure
#
# Drives a simple rectangular patrol route along four ground waypoints,
# then returns to the launch position.
#
# Sequence: connect -> upload mission -> preflight -> arm ->
#           switch to mission mode -> drive route -> RTL -> disarm
#
# Mission commands used:
#   NAV_WAYPOINT (16)           - drive to ground waypoints
#   DO_CHANGE_SPEED (178)       - set driving speed
#   NAV_RETURN_TO_LAUNCH (20)   - return home
# ============================================================================

load_utility '<%= target_name %>/procedures/utilities/cmd_and_ack.rb'
load_utility '<%= target_name %>/procedures/utilities/upload_mission.rb'
load_utility '<%= target_name %>/procedures/utilities/flight_ops.rb'

# ============================================================================
# 1. VERIFY CONNECTION
# ============================================================================
wait_for_heartbeat

# ============================================================================
# 2. BUILD AND UPLOAD ROVER MISSION
#    Rectangular patrol: 30m north -> 30m east -> 30m south -> 30m west -> RTL
#    All waypoints at ground level (z=0), speed 2 m/s
# ============================================================================
puts "--- Building rover mission ---"

home_lat_e7 = tlm("DRONE GLOBAL_POSITION_INT LAT").to_i
home_lon_e7 = tlm("DRONE GLOBAL_POSITION_INT LON").to_i

# ~30m north = 2700 degE7, ~30m east = 3240 degE7 (at mid-latitudes ~47deg)
wp1_lat = home_lat_e7 + 2700
wp1_lon = home_lon_e7

wp2_lat = wp1_lat
wp2_lon = home_lon_e7 + 3240

wp3_lat = home_lat_e7
wp3_lon = wp2_lon

puts "  Home: lat=#{home_lat_e7 / 10000000.0}, lon=#{home_lon_e7 / 10000000.0}"
puts "  WP1 (30m N):  lat=#{wp1_lat / 10000000.0}, lon=#{wp1_lon / 10000000.0}"
puts "  WP2 (30m NE): lat=#{wp2_lat / 10000000.0}, lon=#{wp2_lon / 10000000.0}"
puts "  WP3 (30m E):  lat=#{wp3_lat / 10000000.0}, lon=#{wp3_lon / 10000000.0}"

mission_items = [
  # seq 0: DO_CHANGE_SPEED (cmd 178) - set ground speed to 2 m/s
  #   param1: 1 = ground speed
  #   param2: speed in m/s
  #   param3: throttle (-1 = no change)
  { seq: 0, frame: 2, command: 178, current: 0, autocontinue: 1,
    param1: 1, param2: 2, param3: -1, param4: 0,
    x: 0, y: 0, z: 0 },

  # seq 1: NAV_WAYPOINT (cmd 16) - drive 30m north
  { seq: 1, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 2, param3: 0, param4: 0,
    x: wp1_lat, y: wp1_lon, z: 0 },

  # seq 2: NAV_WAYPOINT (cmd 16) - drive 30m east
  { seq: 2, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 2, param3: 0, param4: 0,
    x: wp2_lat, y: wp2_lon, z: 0 },

  # seq 3: NAV_WAYPOINT (cmd 16) - drive 30m south (back to home longitude)
  { seq: 3, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 2, param3: 0, param4: 0,
    x: wp3_lat, y: wp3_lon, z: 0 },

  # seq 4: NAV_RETURN_TO_LAUNCH (cmd 20)
  { seq: 4, frame: 2, command: 20, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: 0, y: 0, z: 0 }
]

puts "--- Uploading rover mission (#{mission_items.length} items) ---"
upload_mission(mission_items)

# ============================================================================
# 3-6. PREFLIGHT, LOITER, ARM, START MISSION
# ============================================================================
run_preflight_checks
set_loiter_mode
arm_vehicle(confirm_message: "Confirm: Path is clear and safe to arm for rover mission?")
start_mission_mode

# ============================================================================
# 7. MONITOR MISSION EXECUTION
# ============================================================================

puts "--- Driving to waypoint 1 (30m north) ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 1", 60)
puts "Waypoint 1 reached."

puts "--- Driving to waypoint 2 (30m east) ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 2", 60)
puts "Waypoint 2 reached."

puts "--- Driving to waypoint 3 (30m south) ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 3", 60)
puts "Waypoint 3 reached."

puts "--- Returning to launch ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 4", 60)
puts "Rover returned to launch position."

# ============================================================================
# 8. DISARM
# ============================================================================
disarm_vehicle

puts "=== Rover mission complete ==="
