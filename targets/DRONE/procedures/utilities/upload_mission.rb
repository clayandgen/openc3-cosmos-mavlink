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
    wait_check("DRONE MISSION_REQUEST_INT RECEIVED_COUNT > #{req_count + i}", 5)
    requested_seq = tlm("DRONE MISSION_REQUEST_INT SEQ").to_i
    puts "    Vehicle requested item #{requested_seq}"

    cmd("DRONE MISSION_ITEM_INT with " \
        "SEQ #{item[:seq]}, FRAME #{item[:frame]}, COMMAND #{item[:command]}, " \
        "CURRENT #{item[:current]}, AUTOCONTINUE #{item[:autocontinue]}, " \
        "PARAM1 #{item[:param1]}, PARAM2 #{item[:param2]}, " \
        "PARAM3 #{item[:param3]}, PARAM4 #{item[:param4]}, " \
        "X #{item[:x]}, Y #{item[:y]}, Z #{item[:z]}")
  end

  wait_check("DRONE MISSION_ACK RECEIVED_COUNT > #{ack_count}", 10)
  upload_result = tlm("DRONE MISSION_ACK TYPE")
  raise "Mission upload failed: #{upload_result}" unless upload_result == "MAV_MISSION_ACCEPTED"
  puts "  Mission upload complete!"
end
