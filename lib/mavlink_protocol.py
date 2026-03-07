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
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
import json
import logging
import math

logger = logging.getLogger(__name__)


def sanitize_for_json(obj):
    """
    Recursively sanitize values for JSON serialization.
    Converts NaN and Infinity to None (which becomes null in JSON).
    """
    if isinstance(obj, dict):
        return {k: sanitize_for_json(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [sanitize_for_json(item) for item in obj]
    elif isinstance(obj, (bytes, bytearray)):
        return list(obj)
    elif isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return obj
    else:
        return obj


class MavlinkProtocol(Protocol):
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
        logger.info(f"Initialized MAVLink protocol: sysid={self.gcs_sysid}, compid={self.gcs_compid}")

        # Build message class lookup: "HEARTBEAT" -> MAVLink_heartbeat_message
        self._msg_classes = {}
        for msg_id, cls in mavlink2.mavlink_map.items():
            self._msg_classes[cls.msgname] = cls

        # Buffer for incoming data
        self.buffer = b''

    def reset(self):
        """Reset the protocol state"""
        logger.debug("Protocol reset called")
        super().reset()
        self.buffer = b''
        # Only reinitialize mav if attributes are set (handles parent init calling reset)
        if hasattr(self, 'gcs_sysid') and hasattr(self, 'gcs_compid'):
            self.mav = mavlink2.MAVLink(None, srcSystem=self.gcs_sysid, srcComponent=self.gcs_compid)

    def read_data(self, data, extra=None):
        """
        Read and parse MAVLink packets, return as JSON

        Args:
            data: Raw bytes from tcp/ip interface
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

            # Parse the packet with pymavlink
            try:
                msg = self.mav.parse_char(packet_data)

                if msg is not None:
                    # Filter by system ID if specified
                    if self.target_sysid != 0 and msg.get_srcSystem() != self.target_sysid:
                        continue

                    # Convert to JSON for COSMOS
                    msg_dict = msg.to_dict()

                    # Remove internal pymavlink fields
                    msg_dict.pop('mavpackettype', None)

                    # Add metadata
                    msg_dict['MSGID'] = msg.get_msgId()
                    msg_dict['MSGNAME'] = msg.get_type()
                    msg_dict['SYSTEM_ID'] = msg.get_srcSystem()
                    msg_dict['COMPONENT_ID'] = msg.get_srcComponent()
                    msg_dict['SEQ'] = msg.get_seq()

                    # Sanitize NaN/Infinity values to null for valid JSON
                    msg_dict = sanitize_for_json(msg_dict)

                    # Return as JSON bytes
                    return json.dumps(msg_dict).encode('utf-8'), extra
                else:
                    # Parsing failed - log and continue
                    logger.warning(f"Failed to parse MAVLink packet (len={len(packet_data)})")
                    continue

            except Exception as e:
                logger.warning(f"MAVLink decode error: {e}")
                continue

    def write_data(self, data, extra=None):
        """Convert JSON command to MAVLink v2 binary packet"""
        try:
            if isinstance(data, (bytes, bytearray)):
                cmd = json.loads(data.decode('utf-8'))
            elif isinstance(data, str):
                cmd = json.loads(data)
            else:
                cmd = data

            msg_name = cmd.get('MSGNAME', '').upper()
            msg_class = self._msg_classes.get(msg_name)
            if msg_class is None:
                raise ValueError(f"Unknown MAVLink message type: {msg_name}")

            # For COMMAND_LONG, remap friendly param names (e.g. ARM_1 -> PARAM1)
            if msg_name == 'COMMAND_LONG':
                remapped = {}
                for key, value in cmd.items():
                    upper_key = key.upper()
                    if upper_key[-2:-1] == '_' and upper_key[-1:].isdigit():
                        remapped[f'PARAM{upper_key[-1]}'] = value
                    else:
                        remapped[upper_key] = value
                cmd = remapped

            # Build kwargs matching pymavlink's expected field names
            field_kwargs = {}
            for i, field in enumerate(msg_class.fieldnames):
                field_upper = field.upper()
                if field_upper in cmd:
                    field_kwargs[field] = cmd[field_upper]
                elif field in cmd:
                    field_kwargs[field] = cmd[field]
                else:
                    ftype = msg_class.fieldtypes[i]
                    field_kwargs[field] = '' if 'char' in ftype else 0

            msg = msg_class(**field_kwargs)
            packet = msg.pack(self.mav)
            self.mav.seq = (self.mav.seq + 1) % 256

            return packet, extra

        except Exception as e:
            logger.error(f"Failed to encode MAVLink message: {e}")
            return data, extra
