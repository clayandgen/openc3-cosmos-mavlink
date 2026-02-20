# ============================================================================
# Orbit Inspection Mission Procedure
#
# Flies to a point of interest, orbits it at multiple altitudes for
# simulated infrastructure inspection, then lands at a designated zone.
#
# Sequence: connect -> upload mission -> preflight -> arm ->
#           switch to mission mode -> fly inspection -> land -> disarm
#
# Mission commands used:
#   NAV_TAKEOFF (22)        - take off to inspection altitude
#   NAV_WAYPOINT (16)       - navigate to approach/landing points
#   CONDITION_YAW (115)     - point nose at target during approach
#   NAV_LOITER_TURNS (18)   - orbit the POI with N turns
#   NAV_LOITER_TO_ALT (31)  - loiter while descending to lower altitude
#   NAV_LAND (21)           - precision land at designated location
# ============================================================================

load_utility '<%= target_name %>/procedures/utilities/cmd_and_ack.rb'
load_utility '<%= target_name %>/procedures/utilities/upload_mission.rb'
load_utility '<%= target_name %>/procedures/utilities/flight_ops.rb'

# ============================================================================
# 1. VERIFY CONNECTION
# ============================================================================
wait_for_heartbeat

# ============================================================================
# 2. BUILD AND UPLOAD ORBIT INSPECTION MISSION
#    Approach POI (80m north), orbit at 25m, descend to 15m, orbit again,
#    fly to landing zone, land.
# ============================================================================
puts "--- Building orbit inspection mission ---"

home_lat_e7 = tlm("DRONE GLOBAL_POSITION_INT LAT").to_i
home_lon_e7 = tlm("DRONE GLOBAL_POSITION_INT LON").to_i

# Point of interest: 80m north (~7200 degE7)
poi_lat_e7 = home_lat_e7 + 7200
poi_lon_e7 = home_lon_e7

# Approach point: offset slightly south of POI (approach from south)
approach_lat_e7 = poi_lat_e7 - 2700  # ~30m south of POI
approach_lon_e7 = poi_lon_e7

# Landing zone: 10m north of home (~900 degE7)
lz_lat_e7 = home_lat_e7 + 900
lz_lon_e7 = home_lon_e7

high_alt = 25.0
low_alt = 15.0
orbit_radius = 30.0

puts "  Home: lat=#{home_lat_e7 / 10000000.0}, lon=#{home_lon_e7 / 10000000.0}"
puts "  POI: lat=#{poi_lat_e7 / 10000000.0}, lon=#{poi_lon_e7 / 10000000.0}"
puts "  Landing zone: lat=#{lz_lat_e7 / 10000000.0}, lon=#{lz_lon_e7 / 10000000.0}"

# Calculate yaw from approach point to POI (due north = 0 degrees)
yaw_to_poi = 0.0  # POI is directly north of approach point

mission_items = [
  # seq 0: NAV_TAKEOFF (cmd 22) to 25m
  { seq: 0, frame: 3, command: 22, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: home_lat_e7, y: home_lon_e7, z: high_alt },

  # seq 1: NAV_WAYPOINT (cmd 16) to approach point (50m north, 30m south of POI)
  { seq: 1, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: approach_lat_e7, y: approach_lon_e7, z: high_alt },

  # seq 2: CONDITION_YAW (cmd 115) - face the POI (north = 0 deg)
  #   param1: target angle (degrees)
  #   param2: angular speed (deg/s)
  #   param3: direction (-1=shortest, 1=CW, -1=CCW)
  #   param4: 0=absolute angle, 1=relative
  { seq: 2, frame: 2, command: 115, current: 0, autocontinue: 1,
    param1: yaw_to_poi, param2: 30, param3: -1, param4: 0,
    x: 0, y: 0, z: 0 },

  # seq 3: NAV_LOITER_TURNS (cmd 18) - 2 orbits at 25m, 30m radius around POI
  #   param1: number of turns
  #   param3: radius (positive=CW, negative=CCW)
  #   param4: exit xtrack location (0=center)
  { seq: 3, frame: 3, command: 18, current: 0, autocontinue: 1,
    param1: 2, param2: 0, param3: orbit_radius, param4: 0,
    x: poi_lat_e7, y: poi_lon_e7, z: high_alt },

  # seq 4: NAV_LOITER_TO_ALT (cmd 31) - descend to 15m while loitering
  #   param1: heading required (0=no, 1=yes)
  #   param2: radius (positive=CW, negative=CCW; 0=default loiter radius)
  { seq: 4, frame: 3, command: 31, current: 0, autocontinue: 1,
    param1: 0, param2: orbit_radius, param3: 0, param4: 0,
    x: poi_lat_e7, y: poi_lon_e7, z: low_alt },

  # seq 5: NAV_LOITER_TURNS (cmd 18) - 2 more orbits at 15m (closer inspection)
  { seq: 5, frame: 3, command: 18, current: 0, autocontinue: 1,
    param1: 2, param2: 0, param3: orbit_radius, param4: 0,
    x: poi_lat_e7, y: poi_lon_e7, z: low_alt },

  # seq 6: NAV_WAYPOINT (cmd 16) - fly to landing zone (near home)
  { seq: 6, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: lz_lat_e7, y: lz_lon_e7, z: low_alt },

  # seq 7: NAV_LAND (cmd 21) - land at the landing zone
  #   param1: abort alt (0=use system default)
  #   param2: land mode (0=normal)
  #   param4: yaw angle (degrees)
  { seq: 7, frame: 3, command: 21, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: lz_lat_e7, y: lz_lon_e7, z: 0 }
]

puts "--- Uploading orbit inspection mission (#{mission_items.length} items) ---"
upload_mission(mission_items)

# ============================================================================
# 3-6. PREFLIGHT, LOITER, ARM, START MISSION
# ============================================================================
run_preflight_checks
set_loiter_mode
arm_vehicle(confirm_message: "Confirm: Area is clear and safe to arm for orbit inspection?")
start_mission_mode

# ============================================================================
# 7. MONITOR MISSION EXECUTION
# ============================================================================

puts "--- Waiting for takeoff to #{high_alt.to_i}m ---"
wait_check("DRONE GLOBAL_POSITION_INT RELATIVE_ALT > #{(high_alt - 1) * 1000}", 30)
puts "Reached inspection altitude."

puts "--- Flying to approach point ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 1", 60)
puts "Reached approach point."

puts "--- Orbiting POI at #{high_alt.to_i}m (high altitude pass) ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 3", 120)
puts "High altitude orbit complete."

puts "--- Descending to #{low_alt.to_i}m for close inspection ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 4", 60)
puts "Reached low inspection altitude."

puts "--- Orbiting POI at #{low_alt.to_i}m (close inspection pass) ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 5", 120)
puts "Close inspection orbit complete."

puts "--- Flying to landing zone ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 6", 60)
puts "Reached landing zone."

puts "--- Landing ---"
wait_check("DRONE GLOBAL_POSITION_INT RELATIVE_ALT < 1000", 120)
puts "Vehicle has landed."

# ============================================================================
# 8. DISARM
# ============================================================================
disarm_vehicle

puts "=== Orbit inspection mission complete ==="
