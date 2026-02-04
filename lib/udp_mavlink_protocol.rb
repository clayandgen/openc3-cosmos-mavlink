# encoding: ascii-8bit
# MAVLink v2 Protocol for OpenC3 COSMOS
#
# Handles MAVLink v2 packet framing, CRC validation, and header field filling.
#
# MAVLink v2 Packet Structure (12+ bytes):
#   Byte 0:     Magic (0xFD)
#   Byte 1:     Payload Length
#   Byte 2:     Incompatibility Flags
#   Byte 3:     Compatibility Flags
#   Byte 4:     Sequence Number
#   Byte 5:     System ID
#   Byte 6:     Component ID
#   Byte 7-9:   Message ID (24-bit, little-endian)
#   Byte 10+:   Payload (variable length)
#   Last 2:     CRC-16/MCRF4XX (X.25)

require 'openc3/interfaces/protocols/protocol'
require_relative 'mavlink_crc_table'
require_relative 'mavlink_crc_extra'

module OpenC3
  class UdpMavlinkProtocol < Protocol
    # MAVLink v2 constants
    MAVLINK_STX = 0xFD
    MAVLINK_HEADER_LEN = 10
    MAVLINK_CRC_LEN = 2
    MAVLINK_MAX_PAYLOAD_LEN = 255

    # @param gcs_sysid [Integer] Ground Control Station System ID
    # @param gcs_compid [Integer] Ground Control Station Component ID
    # @param target_sysid [Integer] Target vehicle System ID (for filtering, 0 = all)
    # @param allow_empty_data [true/false/nil] See Protocol#initialize
    def initialize(gcs_sysid = 255, gcs_compid = 0, target_sysid = 0, allow_empty_data = nil)
      super(allow_empty_data)
      @gcs_sysid = Integer(gcs_sysid)
      @gcs_compid = Integer(gcs_compid)
      @target_sysid = Integer(target_sysid)
      @sequence = 0
      @data_buffer = ''
    end

    def reset
      super()
      @data_buffer = ''
      @sequence = 0
    end

    # Calculate CRC-16/MCRF4XX (X.25)
    def crc_calculate(data)
      crc = 0xFFFF
      data.each_byte do |byte|
        crc = ((crc >> 8) & 0xFF) ^ OpenC3::MAVLINK_CRC_TABLE[(crc ^ byte) & 0xFF]
      end
      crc
    end

    # Accumulate CRC with a single byte
    def crc_accumulate(byte, crc)
      ((crc >> 8) & 0xFF) ^ OpenC3::MAVLINK_CRC_TABLE[(crc ^ byte) & 0xFF]
    end

    # Read data - handle packet framing and CRC validation
    def read_data(data, extra = nil)
      @data_buffer << data

      loop do
        # Need at least 1 byte to check magic
        return :STOP, extra if @data_buffer.length < 1

        # Look for MAVLink v2 sync byte (0xFD)
        magic = @data_buffer.getbyte(0)
        unless magic == MAVLINK_STX
          # Not a valid sync byte, discard and try again
          @data_buffer = @data_buffer[1..-1]
          next
        end

        # Need full header to determine packet length
        return :STOP, extra if @data_buffer.length < MAVLINK_HEADER_LEN

        # Get payload length from byte 1
        payload_len = @data_buffer.getbyte(1)

        # Calculate total packet length
        total_len = MAVLINK_HEADER_LEN + payload_len + MAVLINK_CRC_LEN

        # Wait for complete packet
        return :STOP, extra if @data_buffer.length < total_len

        # Extract the complete packet
        packet_data = @data_buffer[0, total_len]
        @data_buffer = @data_buffer[total_len..-1] || ''

        # Validate CRC
        if validate_crc(packet_data)
          # Optionally filter by system ID
          if @target_sysid != 0
            sysid = packet_data.getbyte(5)
            if sysid != @target_sysid
              # Not from our target, continue looking
              next
            end
          end

          return packet_data, extra
        else
          # CRC failed, log and continue looking for valid packets
          Logger.warn("MAVLink CRC validation failed, discarding packet")
          next
        end
      end
    end

    # Write data - fill header fields and calculate CRC
    def write_data(data, extra = nil)
      return super(data, extra) if data.length < MAVLINK_HEADER_LEN

      magic = data.getbyte(0)
      return super(data, extra) unless magic == MAVLINK_STX

      # Fill in sequence number (byte 4)
      data.setbyte(4, @sequence)
      @sequence = (@sequence + 1) & 0xFF

      # Fill in system ID (byte 5) and component ID (byte 6)
      data.setbyte(5, @gcs_sysid)
      data.setbyte(6, @gcs_compid)

      # Calculate and append CRC
      payload_len = data.getbyte(1)
      msg_id = data.getbyte(7) | (data.getbyte(8) << 8) | (data.getbyte(9) << 16)

      # CRC is calculated over bytes 1 through end of payload (excluding magic and CRC)
      crc_data = data[1, MAVLINK_HEADER_LEN - 1 + payload_len]
      crc = crc_calculate(crc_data)

      # Add CRC_EXTRA if available
      if OpenC3::MAVLINK_CRC_EXTRA.key?(msg_id)
        crc = crc_accumulate(OpenC3::MAVLINK_CRC_EXTRA[msg_id], crc)
      end

      # Append CRC (little-endian)
      data = data[0, MAVLINK_HEADER_LEN + payload_len]
      data << [crc & 0xFF, (crc >> 8) & 0xFF].pack('CC')

      return data, extra
    end

    private

    def validate_crc(packet_data)
      payload_len = packet_data.getbyte(1)
      msg_id = packet_data.getbyte(7) | (packet_data.getbyte(8) << 8) | (packet_data.getbyte(9) << 16)

      # Extract received CRC (last 2 bytes, little-endian)
      crc_offset = MAVLINK_HEADER_LEN + payload_len
      received_crc = packet_data.getbyte(crc_offset) | (packet_data.getbyte(crc_offset + 1) << 8)

      # Calculate expected CRC over bytes 1 through end of payload
      crc_data = packet_data[1, MAVLINK_HEADER_LEN - 1 + payload_len]
      calculated_crc = crc_calculate(crc_data)

      # Add CRC_EXTRA if available
      if OpenC3::MAVLINK_CRC_EXTRA.key?(msg_id)
        calculated_crc = crc_accumulate(OpenC3::MAVLINK_CRC_EXTRA[msg_id], calculated_crc)
      else
        # Unknown message ID - skip CRC validation
        Logger.debug("MAVLink unknown message ID #{msg_id}, skipping CRC validation")
        return true
      end

      calculated_crc == received_crc
    end
  end
end
