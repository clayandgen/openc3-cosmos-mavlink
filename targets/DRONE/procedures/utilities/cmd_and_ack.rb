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
