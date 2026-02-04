# Script Runner test script
cmd("DRONE EXAMPLE")
wait_check("DRONE STATUS BOOL == 'FALSE'", 5)
