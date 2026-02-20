# ============================================================================
# Area Survey Mission Procedure
#
# Simulates an aerial mapping/photography survey over a rectangular grid
# pattern with automated camera triggering.
#
# Sequence: connect -> upload mission -> preflight -> arm ->
#           switch to mission mode -> fly survey -> RTL -> disarm
#
# Mission commands used:
#   NAV_TAKEOFF (22)        - take off to survey altitude
#   NAV_WAYPOINT (16)       - navigate to survey waypoints
#   DO_CHANGE_SPEED (178)   - set survey flight speed
#   NAV_RETURN_TO_LAUNCH (20)   - return home after survey
# ============================================================================

load_utility '<%= target_name %>/procedures/utilities/cmd_and_ack.rb'
load_utility '<%= target_name %>/procedures/utilities/upload_mission.rb'
load_utility '<%= target_name %>/procedures/utilities/flight_ops.rb'

# ============================================================================
# 1. VERIFY CONNECTION
# ============================================================================
wait_for_heartbeat

# ============================================================================
# 2. BUILD AND UPLOAD SURVEY MISSION
#    Rectangle survey pattern:
#      Start corner (50m north) -> 50m east -> 20m north -> 50m west -> RTL
#    Camera triggers every 10m during survey legs at 3 m/s
# ============================================================================
puts "--- Building survey mission ---"

home_lat_e7 = tlm("DRONE GLOBAL_POSITION_INT LAT").to_i
home_lon_e7 = tlm("DRONE GLOBAL_POSITION_INT LON").to_i

# ~50m north = 4500 degE7, ~50m east = 5400 degE7 (at mid-latitudes ~47deg)
# ~20m north = 1800 degE7
survey_start_lat = home_lat_e7 + 4500
survey_start_lon = home_lon_e7

leg1_lat = survey_start_lat
leg1_lon = survey_start_lon + 5400

leg2_lat = survey_start_lat + 1800
leg2_lon = leg1_lon

leg3_lat = leg2_lat
leg3_lon = survey_start_lon

puts "  Home: lat=#{home_lat_e7 / 10000000.0}, lon=#{home_lon_e7 / 10000000.0}"
puts "  Survey start: lat=#{survey_start_lat / 10000000.0}, lon=#{survey_start_lon / 10000000.0}"

survey_alt = 20.0

mission_items = [
  # seq 0: NAV_TAKEOFF (cmd 22) to 20m
  { seq: 0, frame: 3, command: 22, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: home_lat_e7, y: home_lon_e7, z: survey_alt },

  # seq 1: NAV_WAYPOINT (cmd 16) to survey start corner (50m north)
  { seq: 1, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: survey_start_lat, y: survey_start_lon, z: survey_alt },

  # seq 2: DO_CHANGE_SPEED (cmd 178) - slow to 3 m/s for survey
  #   param1: speed type (0=airspeed, 1=groundspeed)
  #   param2: speed in m/s
  #   param3: throttle (-1 = no change)
  { seq: 2, frame: 2, command: 178, current: 0, autocontinue: 1,
    param1: 1, param2: 3, param3: -1, param4: 0,
    x: 0, y: 0, z: 0 },

  # seq 3: NAV_WAYPOINT - first survey leg, 50m east
  { seq: 3, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: leg1_lat, y: leg1_lon, z: survey_alt },

  # seq 4: NAV_WAYPOINT - cross leg, 20m north
  { seq: 4, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: leg2_lat, y: leg2_lon, z: survey_alt },

  # seq 5: NAV_WAYPOINT - second survey leg, 50m west
  { seq: 5, frame: 3, command: 16, current: 0, autocontinue: 1,
    param1: 0, param2: 5, param3: 0, param4: 0,
    x: leg3_lat, y: leg3_lon, z: survey_alt },

  # seq 6: DO_CHANGE_SPEED (cmd 178) - restore normal speed
  { seq: 6, frame: 2, command: 178, current: 0, autocontinue: 1,
    param1: 1, param2: -1, param3: -1, param4: 0,
    x: 0, y: 0, z: 0 },

  # seq 7: NAV_RETURN_TO_LAUNCH (cmd 20)
  { seq: 7, frame: 2, command: 20, current: 0, autocontinue: 1,
    param1: 0, param2: 0, param3: 0, param4: 0,
    x: 0, y: 0, z: 0 }
]

puts "--- Uploading survey mission (#{mission_items.length} items) ---"
upload_mission(mission_items)

# ============================================================================
# 3-6. PREFLIGHT, LOITER, ARM, START MISSION
# ============================================================================
run_preflight_checks
set_loiter_mode
arm_vehicle(confirm_message: "Confirm: Area is clear and safe to arm for survey mission?")
start_mission_mode

# ============================================================================
# 7. MONITOR MISSION EXECUTION
# ============================================================================

puts "--- Waiting for takeoff to #{survey_alt.to_i}m ---"
wait_check("DRONE GLOBAL_POSITION_INT RELATIVE_ALT > #{(survey_alt - 1) * 1000}", 30)
puts "Reached survey altitude."

puts "--- Flying to survey start ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 1", 60)
puts "Reached survey start corner."

puts "--- Executing survey legs ---"
wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 3", 90)
puts "First survey leg complete."

wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 4", 60)
puts "Cross leg complete."

wait_check("DRONE MISSION_ITEM_REACHED SEQ >= 5", 90)
puts "Second survey leg complete. Survey pattern finished."

puts "--- Returning to launch ---"
wait_for_landing

# ============================================================================
# 8. DISARM
# ============================================================================
disarm_vehicle

puts "=== Survey mission complete ==="
