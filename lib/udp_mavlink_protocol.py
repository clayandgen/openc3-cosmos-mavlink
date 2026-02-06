# encoding: utf-8
"""
MAVLink v2 Protocol for OpenC3 COSMOS (Python + pymavlink)

This protocol uses pymavlink to handle all MAVLink complexity:
- Packet framing and sync detection
- CRC validation with CRC_EXTRA
- MAVLink v2 packet truncation
- Field extraction and byte ordering

The protocol outputs JSON that COSMOS reads using JsonAccessor.
"""

from openc3.interfaces.protocols.protocol import Protocol
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import json
import struct


class UdpMavlinkProtocol(Protocol):
    """MAVLink v2 Protocol using pymavlink"""

    # MAVLink v2 constants
    MAVLINK_STX = 0xFD
    MAVLINK_HEADER_LEN = 10
    MAVLINK_CRC_LEN = 2

    def __init__(self, gcs_sysid=255, gcs_compid=0, target_sysid=0, allow_empty_data=None):
        super().__init__(allow_empty_data)
        self.gcs_sysid = int(gcs_sysid)
        self.gcs_compid = int(gcs_compid)
        self.target_sysid = int(target_sysid)

        # MAVLink parser
        self.mav = mavlink2.MAVLink(None, srcSystem=self.gcs_sysid, srcComponent=self.gcs_compid)

        # Buffer for incoming data
        self.buffer = b''

    def reset(self):
        """Reset the protocol state"""
        super().reset()
        self.buffer = b''
        # Only reinitialize mav if attributes are set (handles parent init calling reset)
        if hasattr(self, 'gcs_sysid') and hasattr(self, 'gcs_compid'):
            self.mav = mavlink2.MAVLink(None, srcSystem=self.gcs_sysid, srcComponent=self.gcs_compid)

    def read_data(self, data, extra=None):
        """
        Read and parse MAVLink packets, return as JSON

        Args:
            data: Raw bytes from UDP interface
            extra: Extra data (unused)

        Returns:
            Tuple of (packet_data, extra) or ('STOP', extra) if need more data
        """
        self.buffer += data

        while True:
            # Need at least 1 byte to check magic
            if len(self.buffer) < 1:
                return 'STOP', extra

            # Look for MAVLink v2 sync byte (0xFD)
            if self.buffer[0] != self.MAVLINK_STX:
                # Not a valid sync byte, discard and try again
                self.buffer = self.buffer[1:]
                continue

            # Need full header to determine packet length
            if len(self.buffer) < self.MAVLINK_HEADER_LEN:
                return 'STOP', extra

            # Get payload length from byte 1
            payload_len = self.buffer[1]

            # Calculate total packet length
            total_len = self.MAVLINK_HEADER_LEN + payload_len + self.MAVLINK_CRC_LEN

            # Wait for complete packet
            if len(self.buffer) < total_len:
                return 'STOP', extra

            # Extract the complete packet
            packet_data = self.buffer[:total_len]
            self.buffer = self.buffer[total_len:]

            # Parse the packet with pymavlink for validation and conversion
            try:
                # Feed the complete packet to pymavlink
                msg = None
                for byte in packet_data:
                    msg = self.mav.parse_char(bytes([byte]))

                if msg is not None:
                    # Filter by system ID if specified
                    if self.target_sysid != 0:
                        if msg.get_srcSystem() != self.target_sysid:
                            # Not from our target, continue looking
                            continue

                    # Convert to JSON for COSMOS
                    msg_dict = msg.to_dict()

                    # Add metadata
                    msg_dict['MSGID'] = msg.get_msgId()
                    msg_dict['MSGNAME'] = msg.get_type()
                    msg_dict['SYSTEM_ID'] = msg.get_srcSystem()
                    msg_dict['COMPONENT_ID'] = msg.get_srcComponent()

                    # Convert to JSON string
                    json_str = json.dumps(msg_dict)

                    # Return JSON as bytes for COSMOS to parse
                    return json_str.encode('utf-8'), extra
                else:
                    # Failed to parse, continue looking for next packet
                    continue

            except Exception as e:
                # Parse/validation error, continue looking for valid packets
                continue

    def write_data(self, data, extra=None):
        """
        Write MAVLink packets

        Args:
            data: Command data (JSON or dict)
            extra: Extra data (unused)

        Returns:
            Tuple of (packet_bytes, extra)
        """
        try:
            # Parse command data (could be JSON string or dict)
            if isinstance(data, bytes):
                cmd = json.loads(data.decode('utf-8'))
            elif isinstance(data, str):
                cmd = json.loads(data)
            else:
                cmd = data

            cmd_type = cmd.get('MSGNAME', '').upper()

            # Handle different message types
            if cmd_type == 'COMMAND_LONG':
                msg = self.mav.command_long_encode(
                    target_system=cmd.get('target_system', 1),
                    target_component=cmd.get('target_component', 1),
                    command=cmd.get('command', 0),
                    confirmation=cmd.get('confirmation', 0),
                    param1=cmd.get('param1', 0.0),
                    param2=cmd.get('param2', 0.0),
                    param3=cmd.get('param3', 0.0),
                    param4=cmd.get('param4', 0.0),
                    param5=cmd.get('param5', 0.0),
                    param6=cmd.get('param6', 0.0),
                    param7=cmd.get('param7', 0.0)
                )
            elif cmd_type == 'HEARTBEAT':
                msg = self.mav.heartbeat_encode(
                    type=cmd.get('type', mavlink2.MAV_TYPE_GCS),
                    autopilot=cmd.get('autopilot', mavlink2.MAV_AUTOPILOT_INVALID),
                    base_mode=cmd.get('base_mode', 0),
                    custom_mode=cmd.get('custom_mode', 0),
                    system_status=cmd.get('system_status', mavlink2.MAV_STATE_ACTIVE)
                )
            else:
                # Unknown command type, pass through if already binary
                if isinstance(data, bytes):
                    return data, extra
                else:
                    raise ValueError(f"Unknown command type: {cmd_type}")

            # Pack message and return
            packet = msg.pack(self.mav)
            return packet, extra

        except Exception as e:
            print(f"Error encoding command: {e}")
            # Return original data if encoding fails
            return data, extra
