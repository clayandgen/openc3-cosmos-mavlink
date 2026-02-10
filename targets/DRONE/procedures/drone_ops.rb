# ============================================================================
# Drone Operations Procedure
#
# A typical guided-mode flight sequence for an ArduPilot drone in Gazebo sim.
# Sequence: connect -> stream setup -> preflight -> arm -> takeoff ->
#           fly waypoints -> return to launch -> land -> disarm
#
# ArduPilot custom_mode values (GUIDED=4, LOITER=5, RTL=6, LAND=9)
# base_mode bit 7 (0x80 / 128) = armed flag
# ============================================================================

# ---------------------------------------------------------------------------
# Helper: send a command and verify the ACK comes back accepted
# ---------------------------------------------------------------------------
def cmd_and_ack(cmd_name, timeout: 5, **params)
  if params.empty?
    cmd("<%= target_name %> #{cmd_name}")
  else
    param_str = params.map { |k, v| "#{k} #{v}" }.join(", ")
    cmd("<%= target_name %> #{cmd_name} with #{param_str}")
  end
  wait_check("<%= target_name %> COMMAND_ACK RESULT == 'MAV_RESULT_ACCEPTED'", timeout)
end

# ============================================================================
# 1. VERIFY CONNECTION - wait for a heartbeat from the vehicle
# ============================================================================
puts "--- Waiting for vehicle heartbeat ---"
wait_check("<%= target_name %> HEARTBEAT SYSTEM_STATUS == 'MAV_STATE_STANDBY'", 30)
puts "Heartbeat received. Vehicle is in STANDBY."

# ============================================================================
# 2. REQUEST TELEMETRY STREAMS at useful rates
#    SET_MESSAGE_INTERVAL: param1=msg_id, param2=interval_us (-1=disable, 0=default)
# ============================================================================
puts "--- Configuring telemetry streams ---"

# GLOBAL_POSITION_INT (33) at 4 Hz (250000 us)
cmd_and_ack("SET_MESSAGE_INTERVAL", MESSAGE_ID: 33, INTERVAL: 250000)

# ATTITUDE (30) at 10 Hz (100000 us)
cmd_and_ack("SET_MESSAGE_INTERVAL", MESSAGE_ID: 30, INTERVAL: 100000)

# SYS_STATUS (1) at 1 Hz (1000000 us)
cmd_and_ack("SET_MESSAGE_INTERVAL", MESSAGE_ID: 1, INTERVAL: 1000000)

# BATTERY_STATUS (147) at 1 Hz
cmd_and_ack("SET_MESSAGE_INTERVAL", MESSAGE_ID: 147, INTERVAL: 1000000)

puts "Telemetry streams configured."

# ============================================================================
# 3. PREFLIGHT CHECKS
# ============================================================================
puts "--- Running pre-arm checks ---"
cmd_and_ack("RUN_PREARM_CHECKS", timeout: 10)
puts "Pre-arm checks passed."

# ============================================================================
# 4. SET MODE TO GUIDED
#    ArduPilot: MODE=1 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), CUSTOM_MODE=4 (GUIDED)
# ============================================================================
puts "--- Setting GUIDED mode ---"
cmd_and_ack("DO_SET_MODE", MODE: 1, CUSTOM_MODE: 4)
wait_check("<%= target_name %> HEARTBEAT CUSTOM_MODE == 4", 5)
puts "Vehicle is in GUIDED mode."

# ============================================================================
# 5. ARM THE VEHICLE
#    ARM=1 to arm, FORCE=0 (respect safety checks)
# ============================================================================
puts "--- Arming vehicle ---"
prompt("Confirm: Area is clear and safe to arm?")

cmd_and_ack("COMPONENT_ARM_DISARM", ARM: 1, FORCE: 0)

# Verify armed via base_mode bit 7 (value >= 128 means armed)
wait_check("<%= target_name %> HEARTBEAT BASE_MODE >= 128", 5)
puts "Vehicle is ARMED."

# ============================================================================
# 6. TAKEOFF to 10 meters
# ============================================================================
puts "--- Taking off to 10m ---"
cmd_and_ack("NAV_TAKEOFF", ALTITUDE: 10.0, timeout: 5)

# Wait until the vehicle reaches roughly 10m (relative_alt is in mm)
wait_check("<%= target_name %> GLOBAL_POSITION_INT RELATIVE_ALT > 9000", 30)
puts "Reached target altitude."
wait(2) # Let it stabilize

# ============================================================================
# 7. FLY TO WAYPOINT
#    Navigate to a point ~50m north of home (adjust coords for your sim)
#    Note: for a real flight, set LATITUDE/LONGITUDE to actual GPS coords.
#    In Gazebo default worlds, home is often near (lat=0, lon=0) or a
#    specific location depending on the world file.
# ============================================================================
puts "--- Flying to Waypoint 1 ---"

# Read current position (converted to degrees by dege7_conversion)
current_lat = tlm("<%= target_name %> GLOBAL_POSITION_INT LAT")  # degrees
current_lon = tlm("<%= target_name %> GLOBAL_POSITION_INT LON")  # degrees

# Move ~50m north (approx 0.00045 degrees latitude)
wp1_lat = (current_lat + 0.00045).round(6)
wp1_lon = current_lon.round(6)
wp1_alt = 10.0

puts "  Current position: lat=#{current_lat}, lon=#{current_lon}"
puts "  Waypoint 1:       lat=#{wp1_lat}, lon=#{wp1_lon}, alt=#{wp1_alt}m"

cmd_and_ack("NAV_WAYPOINT",
  HOLD: 0,
  ACCEPT_RADIUS: 2.0,
  PASS_RADIUS: 0,
  YAW: 0,        # NaN would keep current heading, 0 = face north
  LATITUDE: wp1_lat,
  LONGITUDE: wp1_lon,
  ALTITUDE: wp1_alt)

# Wait for the vehicle to get near the waypoint (crude check via position)
puts "  Waiting to arrive at waypoint..."
wait(15)
puts "Waypoint 1 reached."

# ============================================================================
# 8. LOITER for 10 seconds at current position
# ============================================================================
puts "--- Loitering for 10 seconds ---"
wait(10)
puts "Loiter complete."

# ============================================================================
# 9. RETURN TO LAUNCH
# ============================================================================
puts "--- Returning to launch ---"
cmd_and_ack("NAV_RETURN_TO_LAUNCH")

# RTL sets custom_mode = 6 on ArduPilot
wait_check("<%= target_name %> HEARTBEAT CUSTOM_MODE == 6", 10)
puts "RTL mode active. Heading home..."

# Wait for altitude to come down near ground (< 1m = 1000mm)
wait_check("<%= target_name %> GLOBAL_POSITION_INT RELATIVE_ALT < 1000", 60)
puts "Vehicle has landed."

# ============================================================================
# 10. DISARM
# ============================================================================
puts "--- Disarming vehicle ---"
wait(3) # Let it settle on ground

cmd_and_ack("COMPONENT_ARM_DISARM", ARM: 0, FORCE: 0, timeout: 10)
wait_check("<%= target_name %> HEARTBEAT BASE_MODE < 128", 5)
puts "Vehicle is DISARMED."

puts "=== Drone operation complete ==="
