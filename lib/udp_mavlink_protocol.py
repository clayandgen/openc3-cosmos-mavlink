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
import time


class UdpMavlinkProtocol(Protocol):
    """MAVLink v2 Protocol using pymavlink"""

    # MAVLink v2 constants
    MAVLINK_STX = 0xFD
    MAVLINK_HEADER_LEN = 10
    MAVLINK_CRC_LEN = 2

    # Metadata keys added by this protocol (not MAVLink fields)
    METADATA_KEYS = {'MSGID', 'MSGNAME', 'SYSTEM_ID', 'COMPONENT_ID', 'mavlink_type'}

    def __init__(self, gcs_sysid=255, gcs_compid=0, target_sysid=0, allow_empty_data=None):
        super().__init__(allow_empty_data)
        self.gcs_sysid = int(gcs_sysid)
        self.gcs_compid = int(gcs_compid)
        self.target_sysid = int(target_sysid)

        # MAVLink parser
        self.mav = mavlink2.MAVLink(None, srcSystem=self.gcs_sysid, srcComponent=self.gcs_compid)

        # Build message class lookup: "HEARTBEAT" -> MAVLink_heartbeat_message, etc.
        self._msg_classes = {}
        for msg_id, cls in mavlink2.mavlink_map.items():
            self._msg_classes[cls.msgname] = cls

        # Buffer for incoming data
        self.buffer = b''

        # Error tracking
        self.error_count = 0

    def reset(self):
        """Reset the protocol state"""
        super().reset()
        self.buffer = b''
        self.error_count = 0
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
                # Track errors before parsing
                errors_before = self.mav.total_receive_errors

                # Feed the complete packet to pymavlink
                msg = None
                for byte in packet_data:
                    msg = self.mav.parse_char(bytes([byte]))

                # Check if an error occurred during parsing
                errors_after = self.mav.total_receive_errors

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
                    # Failed to parse - check if pymavlink detected an error
                    if errors_after > errors_before:
                        # pymavlink detected an error
                        self.error_count += 1

                        # Try to extract system/component IDs from raw packet (bytes 5-6)
                        try:
                            system_id = packet_data[5] if len(packet_data) > 5 else 0
                            component_id = packet_data[6] if len(packet_data) > 6 else 0
                        except:
                            system_id = 0
                            component_id = 0

                        # Determine error type from pymavlink state
                        if self.mav.have_prefix_error:
                            error_type = 'PREFIX_ERROR'
                            error_message = 'Invalid MAVLink packet prefix/magic byte'
                        else:
                            error_type = 'CRC_FAILURE'
                            error_message = f'CRC validation failed (pymavlink errors: {errors_after})'

                        # Build error packet as JSON
                        error_packet = {
                            'MSGID': 65535,
                            'MSGNAME': 'MAVLINK_ERROR',
                            'SYSTEM_ID': system_id,
                            'COMPONENT_ID': component_id,
                            'timestamp': int(time.time() * 1000000),  # microseconds
                            'error_type': error_type,
                            'error_message': error_message,
                            'packet_length': len(packet_data),
                            'raw_packet': packet_data.hex()
                        }

                        # Convert to JSON bytes
                        error_json = json.dumps(error_packet).encode('utf-8')

                        # Inject error telemetry into COSMOS
                        try:
                            target_name = getattr(self, 'target_name', 'DRONE')
                            self.inject_tlm(target_name, 'MAVLINK_ERROR', error_json, extra)
                        except Exception as inject_error:
                            print(f"Failed to inject error telemetry: {inject_error}")

                    # Continue looking for next packet
                    continue

            except Exception as e:
                # Parse/validation error - inject error telemetry and continue
                self.error_count += 1

                # Try to extract system/component IDs from raw packet (bytes 5-6)
                try:
                    system_id = packet_data[5] if len(packet_data) > 5 else 0
                    component_id = packet_data[6] if len(packet_data) > 6 else 0
                except:
                    system_id = 0
                    component_id = 0

                # Determine error type
                error_type = type(e).__name__
                if 'crc' in str(e).lower() or 'checksum' in str(e).lower():
                    error_type = 'CRC_FAILURE'
                elif 'parse' in str(e).lower():
                    error_type = 'PARSE_ERROR'

                # Include pymavlink error stats
                error_message = f"{str(e)} (pymavlink total errors: {self.mav.total_receive_errors})"

                # Build error packet as JSON
                error_packet = {
                    'MSGID': 65535,
                    'MSGNAME': 'MAVLINK_ERROR',
                    'SYSTEM_ID': system_id,
                    'COMPONENT_ID': component_id,
                    'timestamp': int(time.time() * 1000000),  # microseconds
                    'error_type': error_type,
                    'error_message': error_message,
                    'packet_length': len(packet_data),
                    'raw_packet': packet_data.hex()
                }

                # Convert to JSON bytes
                error_json = json.dumps(error_packet).encode('utf-8')

                # Inject error telemetry into COSMOS
                try:
                    # Get target name from interface (if available)
                    target_name = getattr(self, 'target_name', 'DRONE')
                    self.inject_tlm(target_name, 'MAVLINK_ERROR', error_json, extra)
                except Exception as inject_error:
                    # If inject fails, at least log it
                    print(f"Failed to inject error telemetry: {inject_error}")

                # Continue looking for valid packets
                continue

    def write_data(self, data, extra=None):
        """
        Convert a JSON MAVLink message to a packed MAVLink v2 binary packet.

        Supports any message type defined in the pymavlink common dialect.
        The JSON must contain a "MSGNAME" field matching a MAVLink message name
        (e.g. "HEARTBEAT", "COMMAND_LONG") and field values keyed by the
        pymavlink field names.

        Args:
            data: Command data as JSON bytes, JSON string, or dict
            extra: Extra data (passed through)

        Returns:
            Tuple of (packet_bytes, extra)
        """
        try:
            # Parse command data
            if isinstance(data, bytes):
                cmd = json.loads(data.decode('utf-8'))
            elif isinstance(data, str):
                cmd = json.loads(data)
            else:
                cmd = data

            msg_name = cmd.get('MSGNAME', '').upper()

            # Look up the message class
            msg_class = self._msg_classes.get(msg_name)
            if msg_class is None:
                raise ValueError(f"Unknown MAVLink message type: {msg_name}")

            # Build field values in the order the message class expects
            field_values = []
            for i, field in enumerate(msg_class.fieldnames):
                if field in cmd:
                    field_values.append(cmd[field])
                else:
                    # Use a zero-value default appropriate for the field type
                    ftype = msg_class.fieldtypes[i]
                    if 'char' in ftype:
                        field_values.append('')
                    else:
                        field_values.append(0)

            # Construct the message and pack it into a full MAVLink v2 packet
            msg = msg_class(*field_values)
            packet = msg.pack(self.mav)
            return packet, extra

        except Exception as e:
            print(f"Error encoding MAVLink message: {e}")
            return data, extra
