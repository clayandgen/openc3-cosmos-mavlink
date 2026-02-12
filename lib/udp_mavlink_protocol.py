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
from pymavlink.dialects.v20 import common as mavlink2
import json
import logging

logger = logging.getLogger(__name__)


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
                # Feed the packet to pymavlink byte-by-byte
                msg = None
                for byte in packet_data:
                    msg = self.mav.parse_char(bytes([byte]))

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
        """
        Convert JSON command to MAVLink v2 binary packet

        Args:
            data: Command data as JSON bytes, JSON string, or dict
            extra: Extra data (passed through)

        Returns:
            Tuple of (packet_bytes, extra)
        """
        try:
            # Parse command data
            if isinstance(data, (bytes, bytearray)):
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
            # pymavlink expects lowercase (param1, target_system)
            # COSMOS sends uppercase (PARAM1, TARGET_SYSTEM)
            field_values = []
            for i, field in enumerate(msg_class.fieldnames):
                field_upper = field.upper()

                if field_upper in cmd:
                    value = cmd[field_upper]
                elif field in cmd:
                    value = cmd[field]
                else:
                    # Default value based on field type
                    ftype = msg_class.fieldtypes[i]
                    value = '' if 'char' in ftype else 0

                field_values.append(value)

            # Pack the message
            msg = msg_class(*field_values)
            packet = msg.pack(self.mav)

            # Manually increment sequence (pymavlink doesn't auto-increment with file=None)
            self.mav.seq = (self.mav.seq + 1) % 256

            logger.info(f"Encoded {msg_name} (seq={packet[4]}, size={len(packet)} bytes)")
            return packet, extra

        except Exception as e:
            logger.error(f"Failed to encode MAVLink message: {e}", exc_info=True)
            return data, extra
