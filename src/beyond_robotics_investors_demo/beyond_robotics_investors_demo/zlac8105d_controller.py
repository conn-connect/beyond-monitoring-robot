from pymodbus.client import ModbusSerialClient
import struct
import time

class ZLAC8105D_Controller:
    def __init__(self, port="/dev/ttyUSB0", motor_id=1):
        self.client = ModbusSerialClient(
            method='rtu',
            port=port,
            baudrate=115200,
            timeout=1
        )
        if not self.client.connect():
            raise ConnectionException("Unable to connect to the motor controller")
        print("Successfully connected to the motor controller")
        self.motor_id = motor_id

        # Register addresses
        self.CONTROL_REG = 0x200E
        self.OPR_MODE = 0x200D
        self.CMD_RPM = 0x2088
        self.FB_RPM = 0x20AB
        self.FAULT = 0x20A5

        # Control commands
        self.ENABLE = 0x0008
        self.DISABLE = 0x0007
        self.ALRM_CLR = 0x0006

        # Operation modes
        self.VEL_CONTROL = 3

    def _write_register(self, address, value):
        result = self.client.write_register(address, value, slave=self.motor_id)
        if result.isError():
            print(f"Error writing to register {address}: {result}")
            return False
        return True

    def enable_motor(self):
        return self._write_register(self.CONTROL_REG, self.ENABLE)

    def disable_motor(self):
        return self._write_register(self.CONTROL_REG, self.DISABLE)

    def set_mode(self, mode):
        return self._write_register(self.OPR_MODE, mode)

    def set_rpm(self, left_rpm, right_rpm):
        left_rpm = int(left_rpm)
        right_rpm = -1 * int(right_rpm)
        packed_value = struct.pack('>hh', left_rpm, right_rpm)
        unpacked_value = struct.unpack('>I', packed_value)[0]
        result = self.client.write_registers(self.CMD_RPM, [unpacked_value >> 16, unpacked_value & 0xFFFF], slave=self.motor_id)
        if result.isError():
            print(f"Error setting RPM: {result}")
            return False
        return True

    def get_rpm(self):
        try:
            response = self.client.read_holding_registers(self.FB_RPM, 2, slave=self.motor_id)
            if response.isError():
                print(f"Error reading RPM: {response}")
                return None, None
            left_rpm = struct.unpack('>h', struct.pack('>H', response.registers[0]))[0] / 10.0
            right_rpm = struct.unpack('>h', struct.pack('>H', response.registers[1]))[0] / -10.0
            return left_rpm, right_rpm
        except Exception as e:
            print(f"Exception in get_rpm: {e}")
            return None, None

    def clear_alarm(self):
        return self._write_register(self.CONTROL_REG, self.ALRM_CLR)

def main():
    controller = ZLAC8105D_Controller()

    try:
        # Enable motors
        print("Enabling motors...")
        if not controller.enable_motor():
            raise Exception("Failed to enable motors")
        time.sleep(1)

        # Set velocity control mode
        print("Setting velocity control mode...")
        if not controller.set_mode(controller.VEL_CONTROL):
            raise Exception("Failed to set velocity control mode")
        time.sleep(1)

        # Set RPM
        print("Setting velocity...")
        if not controller.set_rpm(50, 50):
            raise Exception("Failed to set RPM")
        time.sleep(5)

        # Get current RPM
        left_rpm, right_rpm = controller.get_rpm()
        if left_rpm is not None and right_rpm is not None:
            print(f"Current RPM - Left: {left_rpm}, Right: {right_rpm}")
        else:
            print("Failed to read current RPM")

        # Stop motors
        print("Stopping motors...")
        if not controller.set_rpm(0, 0):
            raise Exception("Failed to stop motors")
        time.sleep(1)

        # Disable motors
        print("Disabling motors...")
        if not controller.disable_motor():
            raise Exception("Failed to disable motors")

        print("Test complete.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        controller.client.close()

if __name__ == "__main__":
    main()
