# encoding: utf-8
"""
MAVLink v2 Protocol for OpenC3 COSMOS (Python + pymavlink)

This protocol uses pymavlink to handle all MAVLink complexity:
- Packet framing and sync detection
- CRC validation with CRC_EXTRA
- MAVLink v2 packet truncation

Read path: raw binary passthrough (header + zero-padded payload) for BinaryAccessor.
Write path: JSON commands encoded via pymavlink.
"""

from openc3.interfaces.protocols.protocol import Protocol
from pymavlink.dialects.v20 import common as mavlink2
import struct
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

        # MAVLink parser (used only for write/command encoding)
        self.mav = mavlink2.MAVLink(None, srcSystem=self.gcs_sysid, srcComponent=self.gcs_compid)
        logger.info(f"Initialized MAVLink protocol: sysid={self.gcs_sysid}, compid={self.gcs_compid}")

        # Build message class lookup: "HEARTBEAT" -> MAVLink_heartbeat_message
        self._msg_classes = {}
        for msg_id, cls in mavlink2.mavlink_map.items():
            self._msg_classes[cls.msgname] = cls

        # Build payload length lookup: msg_id -> full (untruncated) payload size
        self._payload_lengths = {}
        for msg_id, cls in mavlink2.mavlink_map.items():
            self._payload_lengths[msg_id] = struct.calcsize('<' + cls.format)

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
        Read MAVLink packets and return raw binary (header + zero-padded payload).

        COSMOS uses BinaryAccessor to extract fields directly from the binary
        data, so no JSON serialization is needed on the read path.

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

            # Get payload length from byte 1 (may be truncated)
            payload_len = self.buffer[1]

            # Calculate total packet length (header + truncated payload + CRC)
            total_len = self.MAVLINK_HEADER_LEN + payload_len + self.MAVLINK_CRC_LEN

            # Wait for complete packet
            if len(self.buffer) < total_len:
                return 'STOP', extra

            # Extract the complete packet and advance buffer
            pkt = self.buffer[:total_len]
            self.buffer = self.buffer[total_len:]

            # Read message ID from bytes 7-9 (little-endian 24-bit)
            msg_id = pkt[7] | (pkt[8] << 8) | (pkt[9] << 16)

            # Skip unknown message IDs
            expected_len = self._payload_lengths.get(msg_id)
            if expected_len is None:
                logger.warning(f"Unknown MAVLink message ID {msg_id}")
                continue

            # Filter by system ID if specified (byte 5 is system ID)
            if self.target_sysid != 0 and pkt[5] != self.target_sysid:
                continue

            # Extract header (10 bytes) and truncated payload (strip CRC)
            header = pkt[:self.MAVLINK_HEADER_LEN]
            payload = pkt[self.MAVLINK_HEADER_LEN:self.MAVLINK_HEADER_LEN + payload_len]

            # Zero-pad payload to full (untruncated) length
            if len(payload) < expected_len:
                payload = payload + b'\x00' * (expected_len - len(payload))

            return header + payload, extra

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
