load_utility '<%= target_name %>/procedures/utilities/flashlight_payload.rb'

# Servo range: 425 (max forward) to 2000 (max inward)
puts "=== Flashlight Dance ==="

# Start from stowed/off
flashlight_stow
flashlight_off
wait(1)

# Deploy and flash 3 times
flashlight_deploy
wait(0.5)

3.times do |i|
  puts "--- Flash #{i + 1} ---"
  flashlight_on
  wait(0.4)
  flashlight_off
  wait(0.4)
end

# Sweep up slowly through the range
puts "--- Sweeping up ---"
(425..2000).step(100) do |pwm|
  cmd("DRONE DO_SET_SERVO with INSTANCE_1 #{FLASHLIGHT_SERVO_PORT}, PWM_2 #{pwm}")
  wait(0.15)
end
flashlight_on
wait(1)

# Sweep back down
puts "--- Sweeping down ---"
2000.step(425, -100) do |pwm|
  cmd("DRONE DO_SET_SERVO with INSTANCE_1 #{FLASHLIGHT_SERVO_PORT}, PWM_2 #{pwm}")
  wait(0.15)
end
flashlight_off
wait(0.5)

# Strobe while bouncing between deploy and stow
puts "--- Strobe bounce ---"
4.times do
  flashlight_deploy
  flashlight_on
  wait(0.3)
  flashlight_stow
  flashlight_off
  wait(0.3)
end

# Grand finale - rapid strobe at full deploy
puts "--- Finale ---"
flashlight_deploy
wait(0.3)
6.times do
  flashlight_on
  wait(0.15)
  flashlight_off
  wait(0.15)
end

# Clean up
wait(0.5)
flashlight_off
flashlight_stow
puts "=== Flashlight Dance Complete ==="
