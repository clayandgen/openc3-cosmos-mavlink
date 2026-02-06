from openc3.conversions.conversion import Conversion

# Custom conversion class to convert degE7 (degrees * 1e7) to degrees
# Used for MAVLink lat/lon values which are transmitted as integers in degE7 format
class Dege7Conversion(Conversion):
    def __init__(self):
        super().__init__()
        # Result is a floating point number (latitude/longitude in degrees)
        self.converted_type = 'FLOAT'
        # 64-bit float to preserve precision
        self.converted_bit_size = 64

    # @param value [Object] Value in degE7 format (degrees * 10000000)
    # @param packet [Packet] The packet object where the conversion is defined
    # @param buffer [String] The raw packet buffer
    def call(self, value, packet, buffer):
        # Convert from degE7 to degrees by dividing by 10,000,000
        return value / 10000000.0
