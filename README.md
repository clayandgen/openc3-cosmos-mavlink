# OpenC3 COSMOS MAVLink Plugin

![OpenC3 COSMOS MAVLink Plugin](public/store_img.png)

An OpenC3 COSMOS plugin for monitoring and controlling MAVLink-based vehicles (drones, UAVs, rovers, etc.).

## Setting Up / Get Connected

- Connect to your Transmitter (tested with Radiomaster Boxer) via Bluetooth
- Create a bridge to port the serially transmitted data on via Bluetooth to a TCP/IP Server on your host machine. Fill in the XXXXX with your specific transmitter ID. `openc3cli bridgegem openc3-cosmos-bridge-serial router_port=2950 write_port_name=/dev/cu.Matek-mLRS-XXXXX-BT read_port_name=/dev/cu.Matek-mLRS-XXXXX-BT  baud_rate=115200 parity=ODD`
- This creates port 2950 that the transmitter data is now serving data to. COSMOS will connect to this port and other services (like QGroundControl) could also connect to `localhost:2950` directly. In this example, we will port data through COSMOS.
- Deploy COSMOS with 2977 port exposed for tcp
```
  openc3-operator:
    ports:
      - "2977:2977/tcp"
```
- Install the openc3-cosmos-mavlink plugin to COSMOS (more info below)
- Connect QGroundControl to `localhost:2977`, which is a TCP/IP Server that COSMOS is hosting, so all QGroundControl telemetry/commands will go through COSMOS.

## Getting Started

### Prerequisites

- OpenC3 COSMOS 7.x or later
- MAVLink vehicle or simulator (e.g., ArduPilot SITL, PX4, mavproxy)
- Transmitter (tested with Radiomaster Boxer) if using a live vehicle

### Installation

1. **Build the plugin gem:**
   ```bash
   openc3.sh cli rake build VERSION=1.0.0
   ```

2. **Install via COSMOS Admin:**
   - Go to OpenC3 Admin Tool → Plugins Tab
   - Click Install and select `openc3-cosmos-mavlink-1.0.0.gem`
   - Configure parameters:
     - Host: IP address of MAVLink source (e.g., `127.0.0.1`)
     - Port: MAVLink port (based on what you specify in the bridge)
     - System ID Filter: `0` for all, or specific vehicle system ID
   - Click Install

## MAVLink Dialect Support

This plugin uses the **ArduPilotMega** MAVLink dialect, which is a superset that includes messages and commands from:

- `minimal.xml` — Core messages (HEARTBEAT)
- `standard.xml` — Standard MAVLink messages
- `common.xml` — Common message set (GLOBAL_POSITION_INT, COMMAND_LONG, etc.)
- `ArduPilotMega.xml` — ArduPilot-specific messages (AHRS2, EKF_STATUS_REPORT, etc.)
- `uAvionix.xml`, `icarous.xml`, `loweheiser.xml`, `cubepilot.xml` — Vendor-specific extensions

### Regenerating Definitions

The command and telemetry definitions are auto-generated from the MAVLink XML files in `helpers/`. To regenerate after updating XML files:

```bash
cd helpers
ruby mavlink_to_cosmos.rb       # Generates DRONE target cmd/tlm (JSON-based)
ruby mavlink_to_raw_cmds.rb     # Generates DRONE_RAW target commands (binary)
```

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