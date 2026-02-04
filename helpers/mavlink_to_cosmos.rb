#!/usr/bin/env ruby
# MAVLink XML to COSMOS Command/Telemetry Definition Converter
#
# Usage: ruby mavlink_to_cosmos.rb common.xml [output_directory]
#
# This script parses a MAVLink XML definition file and generates
# COSMOS command and telemetry definition files.
#
# - All MAVLink messages become TELEMETRY (received from vehicle)
# - All MAV_CMD_* enum entries become COMMANDS (sent via COMMAND_LONG)

require 'nokogiri'
require 'fileutils'

# Configuration
TARGET_NAME_PLACEHOLDER = '<%= target_name %>'

# MAVLink type to COSMOS type mapping
TYPE_MAP = {
  'uint8_t'  => { cosmos_type: 'UINT', bits: 8 },
  'int8_t'   => { cosmos_type: 'INT', bits: 8 },
  'uint16_t' => { cosmos_type: 'UINT', bits: 16 },
  'int16_t'  => { cosmos_type: 'INT', bits: 16 },
  'uint32_t' => { cosmos_type: 'UINT', bits: 32 },
  'int32_t'  => { cosmos_type: 'INT', bits: 32 },
  'uint64_t' => { cosmos_type: 'UINT', bits: 64 },
  'int64_t'  => { cosmos_type: 'INT', bits: 64 },
  'float'    => { cosmos_type: 'FLOAT', bits: 32 },
  'double'   => { cosmos_type: 'FLOAT', bits: 64 },
  'uint8_t_mavlink_version' => { cosmos_type: 'UINT', bits: 8 },
}

# COMMAND_LONG payload size (33 bytes)
COMMAND_LONG_PAYLOAD_SIZE = 33
COMMAND_LONG_MSG_ID = 76

# Messages to skip (deprecated, WIP, or not useful)
SKIP_MESSAGE_IDS = []

class MavlinkToCosmosConverter
  def initialize(xml_file, target_name = nil)
    @xml_file = xml_file
    @target_name = target_name || TARGET_NAME_PLACEHOLDER
    @xml_dir = File.dirname(File.expand_path(xml_file))
    @processed_files = []
    @enums = {}
    @messages = []
    @mav_commands = []  # MAV_CMD_* entries

    # Process main file and all includes
    process_xml_file(File.expand_path(xml_file))

    extract_mav_commands
  end

  def process_xml_file(xml_file_path)
    # Normalize the path - if already absolute, use as-is; otherwise join with xml_dir
    abs_path = (xml_file_path.start_with?('/') || xml_file_path =~ /^[A-Za-z]:/) ? xml_file_path : File.join(@xml_dir, xml_file_path)
    return if @processed_files.include?(abs_path)
    @processed_files << abs_path

    puts "Processing #{File.basename(abs_path)}..."
    doc = Nokogiri::XML(File.read(abs_path))

    # Process includes first (these are relative paths)
    doc.xpath('//include').each do |include_node|
      include_file = include_node.text.strip
      include_path = File.join(@xml_dir, include_file)
      if File.exist?(include_path)
        process_xml_file(include_file)  # Pass relative path
      else
        puts "  Warning: Include file not found: #{include_file}"
      end
    end

    # Parse enums and messages from this file
    parse_enums_from_doc(doc)
    parse_messages_from_doc(doc)
  end

  # Keep old method names for compatibility (now just wrappers)
  def parse_enums
    # Now handled by process_xml_file
  end

  def parse_messages
    @messages.sort_by! { |m| m[:id] }
    puts "Parsed #{@messages.size} messages total from all files"
  end

  def parse_enums_from_doc(doc)
    doc.xpath('//enum').each do |enum_node|
      enum_name = enum_node['name']
      entries = []

      enum_node.xpath('entry').each do |entry|
        entry_data = {
          value: entry['value']&.to_i,
          name: entry['name'],
          description: entry.xpath('description').text.strip
        }

        # For MAV_CMD entries, also parse param descriptions
        if enum_name == 'MAV_CMD'
          params = []
          (1..7).each do |i|
            param_node = entry.xpath("param[@index='#{i}']").first
            if param_node
              params << {
                index: i,
                label: param_node['label'],
                units: param_node['units'],
                enum: param_node['enum'],
                description: param_node.text.strip,
                reserved: param_node['reserved'] == 'true',
                default: param_node['default']
              }
            else
              params << { index: i, reserved: true, description: "Reserved" }
            end
          end
          entry_data[:params] = params
        end

        entries << entry_data
      end

      # Only add enum if not already present (prevent duplicates from includes)
      unless @enums[enum_name]
        @enums[enum_name] = {
          name: enum_name,
          bitmask: enum_node['bitmask'] == 'true',
          entries: entries
        }
      end
    end
  end

  def parse_messages_from_doc(doc)
    doc.xpath('//message').each do |msg_node|
      msg_id = msg_node['id'].to_i
      msg_name = msg_node['name']

      next if SKIP_MESSAGE_IDS.include?(msg_id)

      description = msg_node.xpath('description').text.strip.gsub(/\s+/, ' ')

      fields = []
      in_extensions = false

      msg_node.children.each do |child|
        if child.name == 'extensions'
          in_extensions = true
          next
        end

        next unless child.name == 'field'

        field_type = child['type']
        field_name = child['name']
        field_units = child['units']
        field_enum = child['enum']
        field_print_format = child['print_format']
        field_description = child.text.strip.gsub(/\s+/, ' ')

        # Handle array types like char[25] or uint8_t[4]
        array_size = nil
        base_type = field_type
        if field_type =~ /^(.+)\[(\d+)\]$/
          base_type = $1
          array_size = $2.to_i
        end

        fields << {
          name: field_name.upcase,
          original_name: field_name,
          type: base_type,
          array_size: array_size,
          units: field_units,
          enum: field_enum,
          print_format: field_print_format,
          description: field_description,
          extension: in_extensions
        }
      end

      # Only add message if not already present (prevent duplicates from includes)
      unless @messages.any? { |m| m[:id] == msg_id }
        @messages << {
          id: msg_id,
          name: msg_name,
          description: description,
          fields: fields
        }
      end
    end
  end

  def extract_mav_commands
    return unless @enums['MAV_CMD']

    @mav_commands = @enums['MAV_CMD'][:entries].select do |entry|
      entry[:name]&.start_with?('MAV_CMD_') && entry[:value]
    end

    @mav_commands.sort_by! { |c| c[:value] }

    # Final summary
    @messages.sort_by! { |m| m[:id] }
    puts "\nParsing complete:"
    puts "  Enums: #{@enums.size}"
    puts "  Messages: #{@messages.size}"
    puts "  Commands: #{@mav_commands.size}"
  end

  def calculate_payload_size(fields)
    base_size = 0
    fields.each do |field|
      next if field[:extension]
      base_size += field_bit_size(field) / 8
    end
    base_size
  end

  def field_bit_size(field)
    if field[:type] == 'char'
      return (field[:array_size] || 1) * 8
    elsif field[:array_size]
      type_info = TYPE_MAP[field[:type]]
      return (type_info&.dig(:bits) || 8) * field[:array_size]
    else
      type_info = TYPE_MAP[field[:type]]
      return type_info&.dig(:bits) || 8
    end
  end

  # Get the wire size of a single element (for sorting)
  def field_element_size(field)
    if field[:type] == 'char'
      return 1  # char arrays are byte-aligned
    else
      type_info = TYPE_MAP[field[:type]]
      return (type_info&.dig(:bits) || 8) / 8
    end
  end

  # Sort fields according to MAVLink v2 wire format rules:
  # - Larger types first (8-byte, 4-byte, 2-byte, 1-byte)
  # - Within same size, maintain original order (stable sort)
  # - Extension fields are sorted separately after base fields
  def sort_fields_for_wire_format(fields)
    base_fields = fields.reject { |f| f[:extension] }
    extension_fields = fields.select { |f| f[:extension] }

    # Sort by element size (descending), then by original index (stable)
    sorted_base = base_fields.each_with_index.sort_by do |field, idx|
      [-field_element_size(field), idx]
    end.map(&:first)

    sorted_extensions = extension_fields.each_with_index.sort_by do |field, idx|
      [-field_element_size(field), idx]
    end.map(&:first)

    sorted_base + sorted_extensions
  end

  def cosmos_type(field)
    if field[:type] == 'char'
      return 'STRING'
    else
      type_info = TYPE_MAP[field[:type]]
      return type_info&.dig(:cosmos_type) || 'UINT'
    end
  end

  # ============================================================================
  # TELEMETRY GENERATION - All messages become telemetry
  # ============================================================================

  def generate_telemetry_file(output_path)
    File.open(output_path, 'w') do |f|
      f.puts "# MAVLink Common Message Set - Telemetry Definitions"
      f.puts "# Auto-generated from #{File.basename(@xml_file)}"
      f.puts "# Reference: https://mavlink.io/en/messages/common.html"
      f.puts "# Note: MAVLink uses LITTLE_ENDIAN byte order"
      f.puts "#"
      f.puts "# All MAVLink messages are telemetry (received from vehicle)"
      f.puts ""

      @messages.each do |msg|
        write_telemetry_packet(f, msg)
        f.puts ""
      end
    end

    puts "Generated telemetry file: #{output_path}"
  end

  def write_telemetry_packet(f, msg)
    f.puts "# " + "=" * 76
    f.puts "# #{msg[:name]} (ID #{msg[:id]})"
    description_lines = word_wrap(msg[:description], 74)
    description_lines.each { |line| f.puts "# #{line}" }
    f.puts "# " + "=" * 76

    short_desc = truncate_description(msg[:description], 100)
    f.puts "TELEMETRY #{TARGET_NAME_PLACEHOLDER} #{msg[:name]} LITTLE_ENDIAN \"#{short_desc}\""

    # Include MAVLink v2 header with explicit bit offsets
    f.puts "  <%= render '_mavlink_v2_header_tlm.txt', locals: {msgid: #{msg[:id]}} %>"

    # Sort fields according to MAVLink v2 wire format (by size)
    sorted_fields = sort_fields_for_wire_format(msg[:fields])

    # IMPORTANT: LITTLE_ENDIAN requires explicit bit offsets (APPEND doesn't work per COSMOS docs)
    # Write fields in wire format order with calculated bit offsets
    has_extensions = sorted_fields.any? { |field| field[:extension] }
    wrote_extension_comment = false
    current_bit_offset = 80  # Start after 10-byte header

    sorted_fields.each do |field|
      if field[:extension] && !wrote_extension_comment
        f.puts "  # MAVLink 2 Extension Fields"
        wrote_extension_comment = true
      end

      write_telemetry_field(f, field, current_bit_offset)
      current_bit_offset += field_bit_size(field)
    end

    # Add checksum at the end
    f.puts "  APPEND_ITEM CHECKSUM 16 UINT \"CRC-16/MCRF4XX checksum\""
    f.puts "    FORMAT_STRING \"0x%04X\""
  end

  def write_telemetry_field(f, field, bit_offset)
    bit_size = field_bit_size(field)
    c_type = cosmos_type(field)
    desc = truncate_description(field[:description], 80)

    # Payload fields use APPEND (only header needs explicit offsets for bitfields)
    if field[:array_size] && field[:type] != 'char'
      item_bits = TYPE_MAP[field[:type]]&.dig(:bits) || 8
      f.puts "  APPEND_ARRAY_ITEM #{field[:name]} #{item_bits} #{c_type} #{bit_size} \"#{desc}\""
    else
      f.puts "  APPEND_ITEM #{field[:name]} #{bit_size} #{c_type} \"#{desc}\""
    end

    # Add format string if specified
    if field[:print_format]
      cosmos_format = convert_print_format(field[:print_format])
      f.puts "    FORMAT_STRING \"#{cosmos_format}\""
    end

    # Add units if specified
    if field[:units]
      units_full, units_abbr = parse_units(field[:units])
      f.puts "    UNITS \"#{units_full}\" #{units_abbr}"
    end

    # Add states for enums (limit to non-bitmask enums with reasonable number of entries)
    if field[:enum] && @enums[field[:enum]]
      enum_data = @enums[field[:enum]]
      if !enum_data[:bitmask] && enum_data[:entries].size <= 20
        enum_data[:entries].first(15).each do |entry|
          next unless entry[:value]
          f.puts "    STATE #{entry[:name]} #{entry[:value]}"
        end
      end
    end
  end

  # ============================================================================
  # COMMAND GENERATION - MAV_CMD_* entries become commands via COMMAND_LONG
  # ============================================================================

  def generate_command_file(output_path)
    File.open(output_path, 'w') do |f|
      f.puts "# MAVLink Common Message Set - Command Definitions"
      f.puts "# Auto-generated from #{File.basename(@xml_file)}"
      f.puts "# Reference: https://mavlink.io/en/messages/common.html"
      f.puts "# Note: MAVLink uses LITTLE_ENDIAN byte order"
      f.puts "#"
      f.puts "# Commands are MAV_CMD_* entries sent via COMMAND_LONG (ID 76)"
      f.puts "# Each command uses the COMMAND_LONG message format with 7 float parameters"
      f.puts ""

      # First, write a generic COMMAND_LONG for advanced users
      write_generic_command_long(f)
      f.puts ""

      # Then write individual commands for each MAV_CMD_*
      @mav_commands.each do |cmd|
        write_mav_command(f, cmd)
        f.puts ""
      end
    end

    puts "Generated command file: #{output_path}"
  end

  def write_generic_command_long(f)
    f.puts "# " + "=" * 76
    f.puts "# COMMAND_LONG (Generic) - Send any MAV_CMD with raw parameters"
    f.puts "# Use this for commands not explicitly defined below"
    f.puts "# " + "=" * 76

    f.puts "COMMAND #{TARGET_NAME_PLACEHOLDER} COMMAND_LONG LITTLE_ENDIAN \"Send a MAVLink command with parameters\""

    # Include MAVLink v2 header with explicit bit offsets
    f.puts "  <%= render '_mavlink_v2_header_cmd.txt', locals: {msgid: #{COMMAND_LONG_MSG_ID}, payload_len: #{COMMAND_LONG_PAYLOAD_SIZE}} %>"

    f.puts "  APPEND_PARAMETER TARGET_SYSTEM 8 UINT 0 255 1 \"Target system ID\""
    f.puts "  APPEND_PARAMETER TARGET_COMPONENT 8 UINT 0 255 1 \"Target component ID\""
    f.puts "  APPEND_PARAMETER COMMAND 16 UINT 0 65535 0 \"MAV_CMD command ID\""

    # Add states for common commands
    f.puts "    STATE MAV_CMD_NAV_WAYPOINT 16"
    f.puts "    STATE MAV_CMD_NAV_RETURN_TO_LAUNCH 20"
    f.puts "    STATE MAV_CMD_NAV_LAND 21"
    f.puts "    STATE MAV_CMD_NAV_TAKEOFF 22"
    f.puts "    STATE MAV_CMD_DO_SET_MODE 176"
    f.puts "    STATE MAV_CMD_COMPONENT_ARM_DISARM 400"
    f.puts "    STATE MAV_CMD_REQUEST_MESSAGE 512"

    f.puts "  APPEND_PARAMETER CONFIRMATION 8 UINT 0 255 0 \"0=First transmission, increment for retries\""
    f.puts "  APPEND_PARAMETER PARAM1 32 FLOAT MIN MAX 0.0 \"Parameter 1\""
    f.puts "  APPEND_PARAMETER PARAM2 32 FLOAT MIN MAX 0.0 \"Parameter 2\""
    f.puts "  APPEND_PARAMETER PARAM3 32 FLOAT MIN MAX 0.0 \"Parameter 3\""
    f.puts "  APPEND_PARAMETER PARAM4 32 FLOAT MIN MAX 0.0 \"Parameter 4\""
    f.puts "  APPEND_PARAMETER PARAM5 32 FLOAT MIN MAX 0.0 \"Parameter 5 (often Latitude)\""
    f.puts "  APPEND_PARAMETER PARAM6 32 FLOAT MIN MAX 0.0 \"Parameter 6 (often Longitude)\""
    f.puts "  APPEND_PARAMETER PARAM7 32 FLOAT MIN MAX 0.0 \"Parameter 7 (often Altitude)\""
    f.puts "  RESPONSE #{TARGET_NAME_PLACEHOLDER} COMMAND_ACK"
  end

  def write_mav_command(f, cmd)
    cmd_name = cmd[:name].sub('MAV_CMD_', '')  # Remove MAV_CMD_ prefix for cleaner names
    cmd_id = cmd[:value]
    description = cmd[:description] || "MAVLink command #{cmd[:name]}"

    f.puts "# " + "=" * 76
    f.puts "# #{cmd_name} (MAV_CMD #{cmd_id})"
    description_lines = word_wrap(description, 74)
    description_lines.each { |line| f.puts "# #{line}" }
    f.puts "# " + "=" * 76

    short_desc = truncate_description(description, 100)
    f.puts "COMMAND #{TARGET_NAME_PLACEHOLDER} #{cmd_name} LITTLE_ENDIAN \"#{short_desc}\""

    # Include MAVLink v2 header with explicit bit offsets
    f.puts "  <%= render '_mavlink_v2_header_cmd.txt', locals: {msgid: #{COMMAND_LONG_MSG_ID}, payload_len: #{COMMAND_LONG_PAYLOAD_SIZE}} %>"

    # COMMAND_LONG structure - use APPEND_* for automatic positioning
    f.puts "  APPEND_PARAMETER TARGET_SYSTEM 8 UINT 0 255 1 \"Target system ID\""
    f.puts "  APPEND_PARAMETER TARGET_COMPONENT 8 UINT 0 255 1 \"Target component ID\""
    f.puts "  APPEND_PARAMETER COMMAND 16 UINT #{cmd_id} #{cmd_id} #{cmd_id} \"#{cmd[:name]}\""
    f.puts "  APPEND_PARAMETER CONFIRMATION 8 UINT 0 255 0 \"0=First transmission, increment for retries\""

    # Write param1-7 based on command definition
    params = cmd[:params] || []
    (1..7).each do |i|
      param = params.find { |p| p[:index] == i } || { reserved: true }
      write_command_param(f, i, param)
    end

    f.puts "  RESPONSE #{TARGET_NAME_PLACEHOLDER} COMMAND_ACK"

    # Mark hazardous commands
    if is_hazardous_command?(cmd_name)
      f.puts "  HAZARDOUS \"#{get_hazard_warning(cmd_name)}\""
    end
  end

  def write_command_param(f, index, param)
    param_name = param[:label]&.upcase&.gsub(/[^A-Z0-9_]/, '_') || "PARAM#{index}"
    param_name = "PARAM#{index}" if param_name.empty?

    # Clean up the param name
    param_name = param_name.gsub(/_+/, '_').gsub(/^_|_$/, '')
    param_name = "PARAM#{index}" if param_name.empty?

    desc = param[:description] || "Parameter #{index}"
    desc = truncate_description(desc, 80)

    # Get default value
    default_val = param[:default] || "0.0"
    default_val = "0.0" if default_val == "NaN" || default_val.to_s.empty?

    if param[:reserved]
      f.puts "  APPEND_PARAMETER PARAM#{index} 32 FLOAT MIN MAX 0.0 \"Reserved (set to 0)\""
      f.puts "    HIDDEN"
    else
      f.puts "  APPEND_PARAMETER #{param_name} 32 FLOAT MIN MAX #{default_val} \"#{desc}\""

      # Add units if specified
      if param[:units]
        units_full, units_abbr = parse_units(param[:units])
        f.puts "    UNITS \"#{units_full}\" #{units_abbr}"
      end

      # Add states for enum parameters
      if param[:enum] && @enums[param[:enum]]
        enum_data = @enums[param[:enum]]
        if !enum_data[:bitmask] && enum_data[:entries].size <= 15
          enum_data[:entries].each do |entry|
            next unless entry[:value]
            f.puts "    STATE #{entry[:name]} #{entry[:value]}"
          end
        end
      end
    end
  end

  def is_hazardous_command?(cmd_name)
    hazardous = %w[
      COMPONENT_ARM_DISARM
      NAV_TAKEOFF
      NAV_LAND
      NAV_RETURN_TO_LAUNCH
      PREFLIGHT_REBOOT_SHUTDOWN
      DO_FLIGHTTERMINATION
      DO_PARACHUTE
      DO_SET_ACTUATOR
      OVERRIDE_GOTO
      MISSION_START
    ]
    hazardous.any? { |h| cmd_name.include?(h) }
  end

  def get_hazard_warning(cmd_name)
    case
    when cmd_name.include?('ARM')
      "Arming enables the motors - ensure area is clear!"
    when cmd_name.include?('TAKEOFF')
      "Vehicle will take off!"
    when cmd_name.include?('LAND')
      "Vehicle will land!"
    when cmd_name.include?('RETURN_TO_LAUNCH')
      "Vehicle will return to launch point!"
    when cmd_name.include?('REBOOT') || cmd_name.include?('SHUTDOWN')
      "This will reboot or shutdown the system!"
    when cmd_name.include?('FLIGHTTERMINATION')
      "This will terminate flight immediately!"
    when cmd_name.include?('PARACHUTE')
      "This will deploy the parachute!"
    else
      "This command may cause vehicle movement!"
    end
  end

  # ============================================================================
  # HELPER METHODS
  # ============================================================================

  def convert_print_format(mavlink_format)
    mavlink_format.gsub('0x%04x', '0x%04X').gsub('0x%08x', '0x%08X')
  end

  def truncate_description(desc, max_len)
    clean = desc.to_s.gsub('"', "'").gsub(/\s+/, ' ').strip
    clean.length > max_len ? clean[0, max_len-3] + "..." : clean
  end

  def parse_units(units_str)
    unit_map = {
      'us' => ['Microseconds', 'us'],
      'ms' => ['Milliseconds', 'ms'],
      's' => ['Seconds', 's'],
      'mV' => ['Millivolts', 'mV'],
      'cA' => ['Centiamps', 'cA'],
      'A' => ['Amps', 'A'],
      '%' => ['Percent', '%'],
      'c%' => ['Centipercent', 'c%'],
      'd%' => ['Decipercent', 'd%'],
      'degE7' => ['Degrees_E7', 'degE7'],
      'deg' => ['Degrees', 'deg'],
      'cdeg' => ['Centidegrees', 'cdeg'],
      'deg/s' => ['Degrees_per_second', 'deg/s'],
      'rad' => ['Radians', 'rad'],
      'rad/s' => ['Radians_per_second', 'rad/s'],
      'mrad/s' => ['Milliradians_per_second', 'mrad/s'],
      'm' => ['Meters', 'm'],
      'cm' => ['Centimeters', 'cm'],
      'mm' => ['Millimeters', 'mm'],
      'm/s' => ['Meters_per_second', 'm/s'],
      'cm/s' => ['Centimeters_per_second', 'cm/s'],
      'mm/s' => ['Millimeters_per_second', 'mm/s'],
      'm/s/s' => ['Meters_per_second_squared', 'm/s/s'],
      'mG' => ['Millig', 'mG'],
      'mT' => ['Millitesla', 'mT'],
      'gauss' => ['Gauss', 'gauss'],
      'mgauss' => ['Milligauss', 'mgauss'],
      'mbar' => ['Millibar', 'mbar'],
      'Pa' => ['Pascal', 'Pa'],
      'hPa' => ['Hectopascal', 'hPa'],
      'K' => ['Kelvin', 'K'],
      'degC' => ['Celsius', 'degC'],
      'cdegC' => ['Centidegrees_Celsius', 'cdegC'],
      'bytes' => ['Bytes', 'B'],
      'bytes/s' => ['Bytes_per_second', 'B/s'],
      'mAh' => ['Milliamp_hours', 'mAh'],
      'hJ' => ['Hectojoules', 'hJ'],
      'rpm' => ['RPM', 'rpm'],
      'Hz' => ['Hertz', 'Hz'],
    }

    if unit_map[units_str]
      unit_map[units_str]
    else
      [units_str.gsub('/', '_per_'), units_str]
    end
  end

  def word_wrap(text, width)
    return [""] if text.nil? || text.empty?
    text.gsub(/(.{1,#{width}})(\s+|$)/, "\\1\n").strip.split("\n")
  end

  def generate_all(output_dir)
    FileUtils.mkdir_p(output_dir)

    tlm_path = File.join(output_dir, 'tlm.txt')
    cmd_path = File.join(output_dir, 'cmd.txt')

    generate_telemetry_file(tlm_path)
    generate_command_file(cmd_path)

    puts ""
    puts "=" * 60
    puts "Generation complete!"
    puts "=" * 60
    puts ""
    puts "Files generated:"
    puts "  Telemetry: #{tlm_path} (#{@messages.size} messages)"
    puts "  Commands:  #{cmd_path} (#{@mav_commands.size} MAV_CMD entries)"
    puts ""
    puts "Note: CRC_EXTRA values are in lib/mavlink_crc_extra.rb"
    puts "      (sourced from MAVLink C library, not generated)"
    puts ""
  end
end

# Main execution
if __FILE__ == $0
  if ARGV.empty?
    puts "MAVLink XML to COSMOS Converter"
    puts ""
    puts "Usage: ruby mavlink_to_cosmos.rb <mavlink.xml> [output_directory]"
    puts ""
    puts "Arguments:"
    puts "  mavlink.xml       - Path to MAVLink XML definition file (e.g., common.xml)"
    puts "  output_directory  - Output directory (default: ./cmd_tlm)"
    puts ""
    puts "Output:"
    puts "  - tlm.txt : All MAVLink messages as COSMOS telemetry"
    puts "  - cmd.txt : All MAV_CMD_* as COSMOS commands"
    exit 1
  end

  xml_file = ARGV[0]
  output_dir = ARGV[1] || './cmd_tlm'

  unless File.exist?(xml_file)
    puts "Error: File not found: #{xml_file}"
    exit 1
  end

  converter = MavlinkToCosmosConverter.new(xml_file)
  converter.generate_all(output_dir)
end
