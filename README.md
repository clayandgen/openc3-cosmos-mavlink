# OpenC3 COSMOS MAVLink Plugin

A comprehensive OpenC3 COSMOS plugin for monitoring and controlling MAVLink-based vehicles (drones, UAVs, rovers, etc.). This plugin provides full support for the MAVLink v2 protocol with JSON-based telemetry and commanding.

## Features

- ✅ **Complete MAVLink v2 Support**: 232 telemetry messages, 166 commands
- ✅ **JSON Accessor Format**: Simplified data handling without binary offsets
- ✅ **Python Protocol Handler**: Uses pymavlink for robust MAVLink parsing
- ✅ **UDP Communication**: Standard MAVLink UDP interface
- ✅ **Pre-built Screens**: Display screens for key telemetry (altitude, attitude, battery, GPS, etc.)
- ✅ **Auto-generation**: Convert MAVLink XML definitions to COSMOS format
- ✅ **CRC Validation**: Full MAVLink v2 CRC_EXTRA validation
- ✅ **System ID Filtering**: Optional filtering by vehicle system ID
- ✅ **Automatic Unit Conversions**: degE7 to degrees for lat/lon values
- ✅ **Error Telemetry**: Inject error packets for CRC failures and parse errors

## Architecture

### Protocol Stack
```
┌─────────────────────────────────────┐
│     COSMOS Interface (JSON)         │
├─────────────────────────────────────┤
│   UdpMavlinkProtocol (Python)       │
│   - JSON ↔ MAVLink conversion       │
│   - CRC validation                  │
│   - Packet framing (0xFD magic)     │
├─────────────────────────────────────┤
│   UDP Socket (14550/14551)          │
├─────────────────────────────────────┤
│   MAVLink Vehicle (Drone/Rover)     │
└─────────────────────────────────────┘
```

### Components

- **Python Protocol** (`lib/udp_mavlink_protocol.py`): Handles MAVLink binary ↔ JSON conversion
- **Ruby Converter** (`helpers/mavlink_to_cosmos.rb`): Generates COSMOS definitions from MAVLink XML
- **Telemetry Definitions** (`targets/DRONE/cmd_tlm/tlm.txt`): 232 MAVLink message definitions
- **Command Definitions** (`targets/DRONE/cmd_tlm/cmd.txt`): 166 MAVLink command definitions
- **Display Screens** (`targets/DRONE/screens/*.txt`): Pre-built telemetry displays

## Getting Started

### Prerequisites

- OpenC3 COSMOS 5.x or later
- Python 3.12+ with pymavlink
- Ruby 3.x (for code generation)
- MAVLink vehicle or simulator (e.g., ArduPilot SITL, PX4, mavproxy)

### Installation

1. **Build the plugin gem:**
   ```bash
   openc3.sh cli rake build VERSION=1.0.0
   ```

2. **Install via COSMOS Admin:**
   - Go to OpenC3 Admin Tool → Plugins Tab
   - Click Install and select `openc3-cosmos-mavlink-1.0.0.gem`
   - Configure parameters:
     - UDP Host: IP address of MAVLink source (e.g., `127.0.0.1`)
     - UDP Port: MAVLink UDP port (typically `14550` or `14551`)
     - System ID Filter: `0` for all, or specific vehicle system ID
   - Click Install

3. **Connect your MAVLink vehicle:**
   - For SITL: `sim_vehicle.py --console --map`
   - For mavproxy: `mavproxy.py --master=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14550`
   - For hardware: Configure vehicle to send MAVLink over UDP

### Quick Test

1. Open COSMOS CmdTlmServer - should see HEARTBEAT messages
2. Open Telemetry Viewer → Select "DRONE HEARTBEAT" screen
3. Verify telemetry is updating
4. Try sending a command: `DRONE HEARTBEAT_CMD`

## Available Screens

Pre-built display screens are available for:

- **altitude.txt** - All altitude measurements (AMSL, terrain, relative, etc.)
- **attitude.txt** - Euler angles and angular rates
- **attitude_quaternion.txt** - Quaternion representation and rates
- **attitude_target.txt** - Commanded attitude targets
- **battery_status.txt** - Battery voltage, current, remaining capacity
- **global_position_int.txt** - GPS position and velocity
- **scaled_pressure.txt** - Barometric pressure sensors
- **servo_output_raw.txt** - All 16 servo/motor outputs
- **sys_status.txt** - System health and sensor status
- **vfr_hud.txt** - VFR HUD display (airspeed, groundspeed, altitude)
- **vibration.txt** - Vibration levels and accelerometer clipping

## Customization

### Adding New MAVLink Messages

If you need to add custom MAVLink messages or update to a newer MAVLink version:

1. Update or add XML files in `helpers/` directory
2. Run the converter:
   ```bash
   ruby helpers/mavlink_to_cosmos.rb helpers/your_dialect.xml targets/DRONE/cmd_tlm
   ```
3. Rebuild and reinstall the plugin

### Creating Custom Screens

Create new screen files in `targets/DRONE/screens/`:

```ruby
SCREEN AUTO AUTO 1.0

VERTICAL
  TITLE "<%= target_name %> My Custom Screen"

  HORIZONTAL
    VERTICALBOX "Section Name"
      LABELVALUE <%= target_name %> PACKET_NAME ITEM_NAME FORMATTED 20
        SETTING TEXTCOLOR var(--color-data-visualization-1)
    END
  END
END
```

### Protocol Configuration

The Python protocol can be configured in `plugin.txt`:

```ruby
INTERFACE DRONE_INT udp_interface.rb 127.0.0.1 14550 14551 nil nil 128 10.0
  PROTOCOL READ UdpMavlinkProtocol lib/udp_mavlink_protocol.py 255 0 1
  # Parameters: gcs_sysid, gcs_compid, target_sysid (0=all)
```

## Development

### Project Structure

```
openc3-cosmos-mavlink/
├── lib/
│   ├── udp_mavlink_protocol.py      # Python MAVLink protocol
│   ├── mavlink_crc_extra.rb         # CRC_EXTRA lookup table
│   └── mavlink_crc_table.rb         # CRC calculation table
├── helpers/
│   ├── mavlink_to_cosmos.rb         # Code generator
│   ├── common.xml                   # MAVLink common messages
│   ├── standard.xml                 # MAVLink standard messages
│   └── minimal.xml                  # MAVLink minimal messages
├── targets/DRONE/
│   ├── cmd_tlm/
│   │   ├── tlm.txt                  # Generated telemetry definitions
│   │   └── cmd.txt                  # Generated command definitions
│   ├── screens/                     # Display screens
│   └── target.txt                   # Target configuration
├── plugin.txt                       # Plugin configuration
├── openc3-cosmos-mavlink.gemspec   # Gem specification
└── README.md                        # This file
```

### Regenerating Definitions

When updating MAVLink XML files or modifying the converter:

```bash
ruby helpers/mavlink_to_cosmos.rb helpers/common.xml targets/DRONE/cmd_tlm
```

This generates:
- `targets/DRONE/cmd_tlm/tlm.txt` (232 messages)
- `targets/DRONE/cmd_tlm/cmd.txt` (166 commands)

### Testing

Test with a MAVLink simulator:

```bash
# ArduPilot SITL
sim_vehicle.py --console --map

# MAVProxy
mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14550
```

## Technical Details

### JSON Format

Telemetry packets are converted to JSON by the Python protocol:

```json
{
  "MSGID": 0,
  "MSGNAME": "HEARTBEAT",
  "SYSTEM_ID": 1,
  "COMPONENT_ID": 1,
  "type": 2,
  "autopilot": 3,
  "base_mode": 81,
  "custom_mode": 0,
  "system_status": 4,
  "mavlink_version": 3
}
```

Commands are sent as JSON and converted to MAVLink binary:

```json
{
  "MSGNAME": "COMMAND_LONG",
  "target_system": 1,
  "target_component": 1,
  "command": 400,
  "confirmation": 0,
  "param1": 1.0,
  "param2": 0.0,
  ...
}
```

### MAVLink v2 Packet Format

The protocol handles the full MAVLink v2 packet structure:

```
Byte 0:      Magic (0xFD)
Byte 1:      Payload Length
Byte 2:      Incompatibility Flags
Byte 3:      Compatibility Flags
Byte 4:      Sequence Number
Byte 5:      System ID
Byte 6:      Component ID
Byte 7-9:    Message ID (24-bit)
Byte 10+:    Payload
Last 2:      CRC-16
```

### Unit Conversions

The plugin includes automatic unit conversions:

**degE7 to Degrees**: Latitude/longitude values are transmitted as integers in degE7 format (degrees × 10,000,000). The `dege7_conversion.py` automatically converts these to floating point degrees.

Example:
- Raw value: 377749000 (degE7 format in JSON)
- Converted value: 37.7749 (displayed as degrees)
- Units: "deg" (degrees, not degE7)

Fields with `units="degE7"` in the MAVLink XML automatically get:
- READ_CONVERSION applied for automatic conversion
- Units changed to "Degrees" "deg" for proper display

## Troubleshooting

### No telemetry received

- Check UDP host/port configuration matches MAVLink source
- Verify firewall allows UDP traffic on configured port
- Check CmdTlmServer logs for connection errors
- Verify MAVLink source is sending to correct IP:port

### CRC validation errors

- Ensure MAVLink XML definitions match vehicle firmware version
- Check that CRC_EXTRA table is up to date
- Some custom messages may require updated XML definitions

### Telemetry parsing errors

- Verify Python protocol is properly installed
- Check that pymavlink is available in COSMOS Python environment
- Review interface logs for Python exceptions

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly with actual MAVLink traffic
5. Submit a pull request

## Resources

- [OpenC3 COSMOS Documentation](https://openc3.com)
- [MAVLink Protocol Specification](https://mavlink.io)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
- [ArduPilot Documentation](https://ardupilot.org)
- [PX4 Documentation](https://docs.px4.io)