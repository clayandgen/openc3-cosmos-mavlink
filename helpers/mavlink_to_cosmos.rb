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
require 'json'

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

  def cosmos_type(field)
    if field[:type] == 'char'
      return 'STRING'
    else
      type_info = TYPE_MAP[field[:type]]
      return type_info&.dig(:cosmos_type) || 'UINT'
    end
  end

  def bit_size(field)
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

  # ============================================================================
  # TELEMETRY GENERATION - All messages become telemetry (JSON format)
  # ============================================================================

  def generate_telemetry_file(output_path)
    File.open(output_path, 'w') do |f|
      f.puts "# MAVLink Common Message Set - Telemetry Definitions (JSON Format)"
      f.puts "# Auto-generated from #{File.basename(@xml_file)}"
      f.puts "# Reference: https://mavlink.io/en/messages/common.html"
      f.puts "#"
      f.puts "# All MAVLink messages are parsed as JSON by the Python protocol"
      f.puts "# Fields are accessed using JsonAccessor"
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
    f.puts "TELEMETRY #{TARGET_NAME_PLACEHOLDER} #{msg[:name]} BIG_ENDIAN \"#{short_desc}\""
    f.puts "  ACCESSOR JsonAccessor"

    # Build template with default values for all fields
    template = build_telemetry_template(msg)
    f.puts "  TEMPLATE '#{template}'"
    f.puts "  # Processed as JSON - fields extracted by JsonAccessor"
    f.puts ""

    # Add metadata fields (injected by protocol)
    f.puts "  APPEND_ID_ITEM MSGID 16 UINT #{msg[:id]} \"Message ID\""
    f.puts "    KEY $.MSGID"
    f.puts ""
    f.puts "  APPEND_ITEM MSGNAME 0 STRING \"Message name\""
    f.puts "    KEY $.MSGNAME"
    f.puts ""
    f.puts "  APPEND_ITEM SYSTEM_ID 8 UINT \"System ID\""
    f.puts "    KEY $.SYSTEM_ID"
    f.puts ""
    f.puts "  APPEND_ITEM COMPONENT_ID 8 UINT \"Component ID\""
    f.puts "    KEY $.COMPONENT_ID"
    f.puts ""

    # Add message-specific fields (in original order, no sorting needed)
    msg[:fields].each do |field|
      if field[:extension]
        f.puts "  # MAVLink 2 Extension Field" if msg[:fields].find { |f| f[:extension] } == field
      end
      write_telemetry_field(f, field)
    end
  end

  def build_telemetry_template(msg)
    # Build JSON object with metadata and all fields with default values
    template_obj = {
      "MSGID" => msg[:id],
      "MSGNAME" => msg[:name],
      "SYSTEM_ID" => 0,
      "COMPONENT_ID" => 0
    }

    # Add all message fields with default values based on type
    msg[:fields].each do |field|
      json_key = field[:original_name]
      default_value = get_field_default_value(field)
      template_obj[json_key] = default_value
    end

    JSON.generate(template_obj)
  end

  def get_field_default_value(field)
    if field[:type] == 'char'
      return ""
    elsif field[:array_size]
      # Return array of zeros for array types
      return Array.new(field[:array_size], 0)
    elsif field[:type].include?('float') || field[:type] == 'double'
      return 0.0
    else
      return 0
    end
  end

  def write_telemetry_field(f, field)
    desc = truncate_description(field[:description], 80)
    json_key = field[:original_name]  # Use original case from XML

    # For JSON accessor, arrays and complex types should use 0 bit size
    # Individual scalar types can specify their size
    if field[:array_size] && field[:type] != 'char'
      # Non-char arrays: use 0 bit size, let JsonAccessor handle it
      bit_size_val = 0
      c_type = 'STRING'  # Arrays are represented as JSON arrays (string format)
    else
      bit_size_val = bit_size(field)
      c_type = cosmos_type(field)
    end

    # Use APPEND_ITEM with JsonAccessor KEY
    f.puts "  APPEND_ITEM #{field[:name]} #{bit_size_val} #{c_type} \"#{desc}\""
    f.puts "    KEY $.#{json_key}"

    # Add conversion for degE7 units (degrees * 1e7 -> degrees)
    if field[:units] == 'degE7'
      f.puts "    READ_CONVERSION dege7_conversion.py"
    end

    # Add units if specified (convert degE7 to degrees for display)
    if field[:units]
      if field[:units] == 'degE7'
        # After conversion, units are degrees not degE7
        f.puts "    UNITS \"Degrees\" deg"
      else
        units_full, units_abbr = parse_units(field[:units])
        f.puts "    UNITS \"#{units_full}\" #{units_abbr}"
      end
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

    f.puts ""
  end

  # ============================================================================
  # COMMAND GENERATION - MAV_CMD_* entries become commands (JSON format)
  # ============================================================================

  def generate_command_file(output_path)
    File.open(output_path, 'w') do |f|
      f.puts "# MAVLink Common Message Set - Command Definitions (JSON Format)"
      f.puts "# Auto-generated from #{File.basename(@xml_file)}"
      f.puts "# Reference: https://mavlink.io/en/messages/common.html"
      f.puts "#"
      f.puts "# Commands are sent as JSON and converted to MAVLink by the Python protocol"
      f.puts "# MAV_CMD_* entries are sent via COMMAND_LONG (ID 76)"
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

    f.puts "COMMAND #{TARGET_NAME_PLACEHOLDER} COMMAND_LONG BIG_ENDIAN \"Send a MAVLink command with parameters\""
    f.puts "  ACCESSOR JsonAccessor"
    f.puts "  TEMPLATE '{\"MSGNAME\":\"COMMAND_LONG\",\"target_system\":1,\"target_component\":1,\"command\":0,\"confirmation\":0,\"param1\":0.0,\"param2\":0.0,\"param3\":0.0,\"param4\":0.0,\"param5\":0.0,\"param6\":0.0,\"param7\":0.0}'"
    f.puts "  # Sent as JSON - converted to MAVLink by protocol"
    f.puts ""
    f.puts "  APPEND_PARAMETER MSGNAME 0 STRING \"COMMAND_LONG\" \"Message type\""
    f.puts ""
    f.puts "  APPEND_PARAMETER TARGET_SYSTEM 8 UINT 1 0 255 \"Target system ID\""
    f.puts ""
    f.puts "  APPEND_PARAMETER TARGET_COMPONENT 8 UINT 1 0 255 \"Target component ID\""
    f.puts ""
    f.puts "  APPEND_PARAMETER COMMAND 16 UINT 0 0 65535 \"MAV_CMD command ID\""
    f.puts "    STATE MAV_CMD_NAV_WAYPOINT 16"
    f.puts "    STATE MAV_CMD_NAV_RETURN_TO_LAUNCH 20"
    f.puts "    STATE MAV_CMD_NAV_LAND 21"
    f.puts "    STATE MAV_CMD_NAV_TAKEOFF 22"
    f.puts "    STATE MAV_CMD_DO_SET_MODE 176"
    f.puts "    STATE MAV_CMD_COMPONENT_ARM_DISARM 400"
    f.puts "    STATE MAV_CMD_REQUEST_MESSAGE 512"
    f.puts ""
    f.puts "  APPEND_PARAMETER CONFIRMATION 8 UINT 0 0 255 \"0=First transmission, increment for retries\""
    f.puts ""

    # PARAM1-7 (each 32-bit float)
    (1..7).each do |i|
      label = case i
      when 5 then "Parameter 5 (often Latitude)"
      when 6 then "Parameter 6 (often Longitude)"
      when 7 then "Parameter 7 (often Altitude)"
      else "Parameter #{i}"
      end
      f.puts "  APPEND_PARAMETER PARAM#{i} 32 FLOAT 0.0 MIN MAX \"#{label}\""
      f.puts ""
    end

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
    f.puts "COMMAND #{TARGET_NAME_PLACEHOLDER} #{cmd_name} BIG_ENDIAN \"#{short_desc}\""
    f.puts "  ACCESSOR JsonAccessor"

    # Build template with default values for all params
    template = build_command_template(cmd)
    f.puts "  TEMPLATE '#{template}'"
    f.puts "  # Sent as JSON - converted to MAVLink COMMAND_LONG by protocol"
    f.puts ""
    f.puts "  APPEND_PARAMETER MSGNAME 0 STRING \"COMMAND_LONG\" \"Message type\""
    f.puts ""
    f.puts "  APPEND_PARAMETER TARGET_SYSTEM 8 UINT 1 0 255 \"Target system ID\""
    f.puts ""
    f.puts "  APPEND_PARAMETER TARGET_COMPONENT 8 UINT 1 0 255 \"Target component ID\""
    f.puts ""
    f.puts "  APPEND_PARAMETER COMMAND 16 UINT #{cmd_id} #{cmd_id} #{cmd_id} \"#{cmd[:name]}\""
    f.puts ""
    f.puts "  APPEND_PARAMETER CONFIRMATION 8 UINT 0 0 255 \"0=First transmission, increment for retries\""
    f.puts ""

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

  def build_command_template(cmd)
    cmd_id = cmd[:value]
    params = cmd[:params] || []

    # Build JSON object with lowercase keys (matching MAVLink convention)
    template_obj = {
      "MSGNAME" => "COMMAND_LONG",
      "target_system" => 1,
      "target_component" => 1,
      "command" => cmd_id,
      "confirmation" => 0
    }

    # Add param1-7 with default values
    (1..7).each do |i|
      param = params.find { |p| p[:index] == i }
      default_val = param&.dig(:default) || "0.0"
      default_val = "0.0" if default_val == "NaN" || default_val.to_s.empty?
      template_obj["param#{i}"] = default_val.to_f
    end

    JSON.generate(template_obj)
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
      f.puts "  APPEND_PARAMETER PARAM#{index} 32 FLOAT 0.0 MIN MAX \"Reserved (set to 0)\""
      f.puts "    HIDDEN"
    else
      f.puts "  APPEND_PARAMETER #{param_name} 32 FLOAT #{default_val} MIN MAX \"#{desc}\""

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

    f.puts ""
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
    puts "Format: JSON accessor (no binary offsets needed)"
    puts "Protocol: Python UdpMavlinkProtocol handles MAVLink encoding/decoding"
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
