#!/usr/bin/env ruby
# MAVLink XML to COSMOS Raw Command Definition Generator
#
# Usage: ruby mavlink_to_raw_cmds.rb [output_file]
#
# Parses all MAVLink XML files in the helpers directory and generates
# a raw command file for the DRONE_RAW target.

require 'nokogiri'
require 'fileutils'

TYPE_SIZES = {
  'uint8_t' => 1, 'int8_t' => 1, 'uint16_t' => 2, 'int16_t' => 2,
  'uint32_t' => 4, 'int32_t' => 4, 'uint64_t' => 8, 'int64_t' => 8,
  'float' => 4, 'double' => 8, 'char' => 1, 'uint8_t_mavlink_version' => 1,
}

xml_dir = File.dirname(File.expand_path(__FILE__))
output_path = ARGV[0] || File.join(xml_dir, '..', 'targets', 'DRONE_RAW', 'cmd_tlm', 'cmd.txt')

processed_files = []
messages = []

def process_xml(path, xml_dir, processed_files, messages)
  abs = File.expand_path(path)
  return if processed_files.include?(abs)
  processed_files << abs

  puts "Processing #{File.basename(abs)}..."
  doc = Nokogiri::XML(File.read(abs))

  doc.xpath('//include').each do |inc|
    inc_path = File.join(xml_dir, inc.text.strip)
    process_xml(inc_path, xml_dir, processed_files, messages) if File.exist?(inc_path)
  end

  doc.xpath('//message').each do |msg_node|
    msg_id = msg_node['id'].to_i
    next if messages.any? { |m| m[:id] == msg_id }

    payload_len = 0
    msg_node.children.each do |child|
      break if child.name == 'extensions'
      next unless child.name == 'field'
      type = child['type']
      base_type, array_size = type =~ /^(.+)\[(\d+)\]$/ ? [$1, $2.to_i] : [type, 1]
      payload_len += (TYPE_SIZES[base_type] || 1) * array_size
    end

    desc = msg_node.xpath('description').text.strip.gsub(/\s+/, ' ').gsub('"', "'")[0..100]
    messages << { id: msg_id, name: msg_node['name'], description: desc, payload_len: payload_len }
  end
end

main_xml = File.join(xml_dir, 'ArduPilotMega.xml')
if File.exist?(main_xml)
  process_xml(main_xml, xml_dir, processed_files, messages)
else
  Dir.glob(File.join(xml_dir, '*.xml')).each { |f| process_xml(f, xml_dir, processed_files, messages) }
end

messages.sort_by! { |m| m[:id] }

FileUtils.mkdir_p(File.dirname(output_path))
File.open(output_path, 'w') do |f|
  f.puts "# MAVLink Raw Command Definitions for DRONE_RAW Target"
  f.puts "# Auto-generated from MAVLink XML definitions"
  f.puts ""
  messages.each do |msg|
    f.puts "COMMAND <%= target_name %> #{msg[:name]} LITTLE_ENDIAN \"#{msg[:description]}\""
    f.puts "  <%= render '_cmd_header.txt', locals: {msgid: #{msg[:id]}, payload_len: #{msg[:payload_len]}} %>"
    f.puts "  APPEND_PARAMETER DATA 0 BLOCK \"\" \"Raw Command Data\""
    f.puts ""
  end
end

puts "Generated #{messages.size} raw commands to #{output_path}"
