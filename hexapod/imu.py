from smbus2 import SMBus
import time
import struct
import math

# IMU Addresses
BNO055_ADDRESS = 0x28

# BNO055 Registers
BNO055_CHIP_ID = 0x00
BNO055_OPR_MODE = 0x3D
BNO055_SYS_TRIGGER = 0x3F
BNO055_PWR_MODE = 0x3E
BNO055_PAGE_ID = 0x07
BNO055_ACCEL_DATA = 0x08
BNO055_GYRO_DATA = 0x14
BNO055_MAG_DATA = 0x0E
BNO055_EULER_DATA = 0x1A
BNO055_QUATERNION_DATA = 0x20

class IMUSensor:
    def __init__(self, bus_number=7):
        self.bus = SMBus(bus_number)
        self.initialize_sensors()
        
    def initialize_sensors(self):
        # Try to initialize BNO055
        try:
            # Check BNO055 chip ID
            chip_id = self.bus.read_byte_data(BNO055_ADDRESS, BNO055_CHIP_ID)
            if chip_id == 0xA0:
                print(f"BNO055 detected with chip ID: 0x{chip_id:02X}")
                
                # Reset BNO055
                self.bus.write_byte_data(BNO055_ADDRESS, BNO055_SYS_TRIGGER, 0x20)
                time.sleep(0.65)  # Wait for reset
                
                # Set to config mode
                self.bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE, 0x00)
                time.sleep(0.05)
                
                # Set to normal power mode
                self.bus.write_byte_data(BNO055_ADDRESS, BNO055_PWR_MODE, 0x00)
                time.sleep(0.01)
                
                # Set to page 0
                self.bus.write_byte_data(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00)
                time.sleep(0.01)
                
                # Set to NDOF mode (fusion)
                self.bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE, 0x0C)
                time.sleep(0.02)
                
                self.bno055_available = True
                print("BNO055 initialized successfully")
            else:
                print(f"Unexpected BNO055 chip ID: 0x{chip_id:02X}")
                self.bno055_available = False
        except Exception as e:
            print(f"BNO055 initialization failed: {e}")
            self.bno055_available = False
            
    def read_data(self):
        if not self.bno055_available:
            return None
            
        data = {
            'accel': {'x': 0, 'y': 0, 'z': 0},
            'gyro': {'x': 0, 'y': 0, 'z': 0},
            'mag': {'x': 0, 'y': 0, 'z': 0},
            'euler': {'heading': 0, 'roll': 0, 'pitch': 0},
            'quaternion': {'w': 0, 'x': 0, 'y': 0, 'z': 0}
        }
        
        # Read accelerometer data (2 bytes each for x, y, z)
        accel_data = self.bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_ACCEL_DATA, 6)
        data['accel']['x'] = struct.unpack('<h', bytes(accel_data[0:2]))[0] / 100.0  # m/s²
        data['accel']['y'] = struct.unpack('<h', bytes(accel_data[2:4]))[0] / 100.0
        data['accel']['z'] = struct.unpack('<h', bytes(accel_data[4:6]))[0] / 100.0
        
        # Read gyroscope data (2 bytes each for x, y, z)
        gyro_data = self.bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_GYRO_DATA, 6)
        data['gyro']['x'] = struct.unpack('<h', bytes(gyro_data[0:2]))[0] / 16.0  # deg/s
        data['gyro']['y'] = struct.unpack('<h', bytes(gyro_data[2:4]))[0] / 16.0
        data['gyro']['z'] = struct.unpack('<h', bytes(gyro_data[4:6]))[0] / 16.0
        
        # Read magnetometer data (2 bytes each for x, y, z)
        mag_data = self.bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_MAG_DATA, 6)
        data['mag']['x'] = struct.unpack('<h', bytes(mag_data[0:2]))[0] / 16.0  # µT
        data['mag']['y'] = struct.unpack('<h', bytes(mag_data[2:4]))[0] / 16.0
        data['mag']['z'] = struct.unpack('<h', bytes(mag_data[4:6]))[0] / 16.0
        
        # Read Euler angles data (2 bytes each for heading, roll, pitch)
        euler_data = self.bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_EULER_DATA, 6)
        data['euler']['heading'] = struct.unpack('<h', bytes(euler_data[0:2]))[0] / 16.0  # degrees
        data['euler']['roll'] = struct.unpack('<h', bytes(euler_data[2:4]))[0] / 16.0
        data['euler']['pitch'] = struct.unpack('<h', bytes(euler_data[4:6]))[0] / 16.0
        
        # Read quaternion data (2 bytes each for w, x, y, z)
        quat_data = self.bus.read_i2c_block_data(BNO055_ADDRESS, BNO055_QUATERNION_DATA, 8)
        data['quaternion']['w'] = struct.unpack('<h', bytes(quat_data[0:2]))[0] / 16384.0
        data['quaternion']['x'] = struct.unpack('<h', bytes(quat_data[2:4]))[0] / 16384.0
        data['quaternion']['y'] = struct.unpack('<h', bytes(quat_data[4:6]))[0] / 16384.0
        data['quaternion']['z'] = struct.unpack('<h', bytes(quat_data[6:8]))[0] / 16384.0
        
        return data
     
    def read_all_data(self):
        result = {}
        
        # BNO055 data
        bno055_data = self.read_data()
        if bno055_data:
            result['bno055'] = bno055_data
        
        return result
    
    def close(self):
        self.bus.close()

def main():
    # Initialize the IMU sensor
    imu = IMUSensor(bus_number=7)
    
    try:
        # Read and print IMU data continuously
        print("Reading IMU data... Press Ctrl+C to exit")
        while True:
            data = imu.read_all_data()
            
            # Print BNO055 data if available
            if 'bno055' in data:
                print("\n=== BNO055 Data ===")
                bno = data['bno055']
                print(f"Accel (m/s²): X={bno['accel']['x']:.2f}, Y={bno['accel']['y']:.2f}, Z={bno['accel']['z']:.2f}")
                print(f"Gyro (deg/s): X={bno['gyro']['x']:.2f}, Y={bno['gyro']['y']:.2f}, Z={bno['gyro']['z']:.2f}")
                print(f"Mag (µT): X={bno['mag']['x']:.2f}, Y={bno['mag']['y']:.2f}, Z={bno['mag']['z']:.2f}")
                print(f"Euler (deg): Heading={bno['euler']['heading']:.2f}, Roll={bno['euler']['roll']:.2f}, Pitch={bno['euler']['pitch']:.2f}")
                print(f"Quaternion: W={bno['quaternion']['w']:.4f}, X={bno['quaternion']['x']:.4f}, Y={bno['quaternion']['y']:.4f}, Z={bno['quaternion']['z']:.4f}")
                     
            time.sleep(0.5)  # Update every 0.5 seconds
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        imu.close()
        print("IMU connection closed")

if __name__ == "__main__":
    main()