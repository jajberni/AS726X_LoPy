"""MicroPython driver for the AS726X spectral sensor"""

from micropython import const
import time
import struct
from Device import Device

__version__ = "0.1.0"

AS726X_ADDR = 0x49 #7-bit unshifted default I2C Address
SENSORTYPE_AS7262 = 0x3E
SENSORTYPE_AS7263 = 0x3F

#Register addresses
AS726x_DEVICE_TYPE = 0x00
AS726x_HW_VERSION = 0x01
AS726x_CONTROL_SETUP = 0x04
AS726x_INT_T = 0x05
AS726x_DEVICE_TEMP = 0x06
AS726x_LED_CONTROL = 0x07

AS72XX_SLAVE_STATUS_REG = 0x00
AS72XX_SLAVE_WRITE_REG = 0x01
AS72XX_SLAVE_READ_REG = 0x02

#The same register locations are shared between the AS7262 and AS7263, they're just called something different
#AS7262 Registers
AS7262_V = 0x08
AS7262_B = 0x0A
AS7262_G = 0x0C
AS7262_Y = 0x0E
AS7262_O = 0x10
AS7262_R = 0x12
AS7262_V_CAL = 0x14
AS7262_B_CAL = 0x18
AS7262_G_CAL = 0x1C
AS7262_Y_CAL = 0x20
AS7262_O_CAL = 0x24
AS7262_R_CAL = 0x28

#AS7263 Registers
AS7263_R = 0x08
AS7263_S = 0x0A
AS7263_T = 0x0C
AS7263_U = 0x0E
AS7263_V = 0x10
AS7263_W = 0x12
AS7263_R_CAL = 0x14
AS7263_S_CAL = 0x18
AS7263_T_CAL = 0x1C
AS7263_U_CAL = 0x20
AS7263_V_CAL = 0x24
AS7263_W_CAL = 0x28

AS72XX_SLAVE_TX_VALID = 0x02
AS72XX_SLAVE_RX_VALID = 0x01

SENSORTYPE_AS7262 = 0x3E
SENSORTYPE_AS7263 = 0x3F

POLLING_DELAY = 5 #Amount of ms to wait between checking for virtual register changes

class AS726X:
    def __init__(self, i2c, address=AS726X_ADDR, mode=3, gain=3,
                 integration_time=50,
                 **kwargs):
        # Check that mode is valid.
        if mode not in [0, 1, 2, 3]:
            raise ValueError(
                'Unexpected mode value {0}. Set mode to 0-3'.format(mode))
        self._mode = mode
        # TODO: Sanitize gain and integration time values
        self._gain = gain
        self._integration_time = integration_time
        self._sensor_version = 0
        # Create I2C device.
        if i2c is None:
            raise ValueError('An I2C object is required.')
        self._device = Device(address, i2c)
        self._i2c = i2c

        # Check and initialize device.
        self.init_device()

    # Read a virtual register from the AS726x
    def virtual_read_register(self, virtual_address):
        # Do a prelim check of the read register
    	status = self._device.readU8(AS72XX_SLAVE_STATUS_REG);
    	if ((status & AS72XX_SLAVE_RX_VALID) != 0): # There is data to be read
    	   _ = self._device.readU8(AS72XX_SLAVE_READ_REG) # Read the byte but do nothing with it

        # Wait for WRITE register to be empty
        while True:
            status = self._device.readU8(AS72XX_SLAVE_STATUS_REG)
            if (status & AS72XX_SLAVE_TX_VALID) == 0:
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(POLLING_DELAY)

        # Send the virtual register address (bit 7 should be 0 to indicate we are reading a register)
        self._device.write8(AS72XX_SLAVE_WRITE_REG, virtual_address)

        # Wait for READ flag to be set
        while True:
            status = self._device.readU8(AS72XX_SLAVE_STATUS_REG)
            if ((status & AS72XX_SLAVE_RX_VALID) != 0): # Data is ready
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(POLLING_DELAY)

        result = self._device.readU8(AS72XX_SLAVE_READ_REG)
        return result

    # Write to a virtual register in the AS726x
    def virtual_write_register(self, virtual_address, value):
        # Wait for WRITE register to be empty
        while True:
            status = self._device.readU8(AS72XX_SLAVE_STATUS_REG)
            if ((status & AS72XX_SLAVE_TX_VALID) == 0):
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(POLLING_DELAY)

        # Send the virtual register address (setting bit 7 to indicate we are writing to a register).
        self._device.write8(AS72XX_SLAVE_WRITE_REG, (virtual_address | 0x80))

        # Wait for WRITE register to be empty
        while True:
            status = self._device.readU8(AS72XX_SLAVE_STATUS_REG)
            if ((status & AS72XX_SLAVE_TX_VALID) == 0):
                break # No inbound TX pending at slave. Okay to write now.
            time.sleep_ms(POLLING_DELAY)

        # Send the data to complete the operation.
        self._device.write8(AS72XX_SLAVE_WRITE_REG, value)


    #Sets the measurement mode
    #Mode 0: Continuous reading of VBGY (7262) / STUV (7263)
    #Mode 1: Continuous reading of GYOR (7262) / RTUX (7263)
    #Mode 2: Continuous reading of all channels (power-on default)
    #Mode 3: One-shot reading of all channels
    def set_measurement_mode(self, mode):
        if (mode > 0b11):
            mode = 0b11

    	# Read, mask/set, write
    	value = self.virtual_read_register(AS726x_CONTROL_SETUP)
    	value = value & 0b11110011
    	value = value | (mode << 2) #Set BANK bits with user's choice
    	self.virtual_write_register(AS726x_CONTROL_SETUP, value)
        self._mode = mode

    #Sets the gain value
    #Gain 0: 1x (power-on default)
    #Gain 1: 3.7x
    #Gain 2: 16x
    #Gain 3: 64x
    def set_gain(self, gain):
        if gain > 0b11:
             gain = 0b11;

    	#Read, mask/set, write
    	value = self.virtual_read_register(AS726x_CONTROL_SETUP)
    	value = value & 0b11001111
    	value = value | (gain << 4) #Set GAIN bits with user's choice
    	self.virtual_write_register(AS726x_CONTROL_SETUP, value)
        self._gain = gain


    #Sets the integration value
    #Give this function a byte from 0 to 255.
    #Time will be 2.8ms * [integration value]
    def set_integration_time(self, integration_time):
        if integration_time > 255:
            integration_time = 255
        self.virtual_write_register(AS726x_INT_T, integration_time)
        self._integration_time = integration_time


    #Set the current limit of bulb/LED.
    #Current 0: 12.5mA
    #Current 1: 25mA
    #Current 2: 50mA
    #Current 3: 100mA
    def set_bulb_current(self, current_level):
        if current_level > 0b11:
            current_level = 0b11
        #Read, mask/set, write
    	value = self.virtual_read_register(AS726x_LED_CONTROL)
    	value = value & 0b11001111
    	value = value | (current_level << 4) #Set ICL_DRV bits with user's choice
    	self.virtual_write_register(AS726x_CONTROL_SETUP, value)


    def enable_bulb(self):
        value = self.virtual_read_register(AS726x_LED_CONTROL)
        value = value | (1 << 3)
        self.virtual_write_register(AS726x_LED_CONTROL, value)

    def disable_bulb(self):
        value = self.virtual_read_register(AS726x_LED_CONTROL)
    	value = value & ~(1 << 3)
    	self.virtual_write_register(AS726x_LED_CONTROL, value)

    # Enable the onboard indicator LED
    def enable_indicator_led(self):
        value = self.virtual_read_register(AS726x_LED_CONTROL)
        value = value | (1 << 0)
        self.virtual_write_register(AS726x_LED_CONTROL, value)

    # Disable the onboard indicator LED
    def disable_indicator_led(self):
        value = self.virtual_read_register(AS726x_LED_CONTROL)
        value = value & ~(1 << 0)
        self.virtual_write_register(AS726x_LED_CONTROL, value)

    # Set the current limit of onboard LED. Default is max 8mA = 0b11
    def set_indicator_current(self, current_level):
        if (current_level > 0b11):
            current_level = 0b11;
    	#Read, mask/set, write
    	value = self.virtual_read_register(AS726x_LED_CONTROL)
    	value = value & 0b11111001; #Clear ICL_IND bits
    	value = value | (current_level << 1); #Set ICL_IND bits with user's choice
    	self.virtual_write_register(AS726x_LED_CONTROL, value)

    def clear_data_available(self):
    	value = self.virtual_read_register(AS726x_CONTROL_SETUP)
    	value = value & ~(1 << 1) #Set the DATA_RDY bit
    	self.virtual_write_register(AS726x_CONTROL_SETUP, value)

    def data_available(self):
        value = self.virtual_read_register(AS726x_CONTROL_SETUP)
        return (value & (1 << 1))

    def get_calibrated_value(self, cal_address):
        b_arr = bytearray(4)
        b_arr[0] = self.virtual_read_register(cal_address + 0);
        b_arr[1] = self.virtual_read_register(cal_address + 1);
        b_arr[2] = self.virtual_read_register(cal_address + 2);
        b_arr[3] = self.virtual_read_register(cal_address + 3);

        return(struct.unpack('>f', b_arr)[0])

    def get_calibrated_values(self):
        calibrated_values = []
        if self._sensor_version == SENSORTYPE_AS7262:
            calibrated_values.append(self.get_calibrated_violet())
            calibrated_values.append(self.get_calibrated_blue())
            calibrated_values.append(self.get_calibrated_green())
            calibrated_values.append(self.get_calibrated_yellow())
            calibrated_values.append(self.get_calibrated_orange())
            calibrated_values.append(self.get_calibrated_red())

        elif self._sensor_version == SENSORTYPE_AS7263:
            calibrated_values.append(self.get_calibrated_R())
            calibrated_values.append(self.get_calibrated_S())
            calibrated_values.append(self.get_calibrated_T())
            calibrated_values.append(self.get_calibrated_U())
            calibrated_values.append(self.get_calibrated_V())
            calibrated_values.append(self.get_calibrated_W())
        return calibrated_values


    def take_measurements(self):
        # Clear DATA_RDY flag when using Mode 3
        self.clear_data_available()

        # Goto mode 3 for one shot measurement of all channels
        self.set_measurement_mode(3);

        #Wait for data to be ready
        while self.data_available() == False:
            time.sleep_ms(POLLING_DELAY)

        #Readings can now be accessed via getViolet(), getBlue(), etc

    def take_measurements_with_bulb(self):
        self.enable_bulb()
        self.take_measurements()
        self.disable_bulb()

    #Returns the temperature in C
    #Pretty inaccurate: +/-8.5C
    def get_temperature(self):
        return (self.virtual_read_register(AS726x_DEVICE_TEMP))

    def soft_reset(self):
        #Read, mask/set, write
    	value = self.virtual_read_register(AS726x_CONTROL_SETUP)
    	value = value | (1 << 7)
    	self.virtual_write_register(AS726x_CONTROL_SETUP, value)
        time.sleep_ms(800)

    def init_device(self):
        self._sensor_version = self.virtual_read_register(AS726x_HW_VERSION)
        if (self._sensor_version != 0x3E) & (self._sensor_version != 0x3F):
            raise ValueError("Wrong sensor version {}. Should be 0x3E or 0x3F".format(self._sensor_version))

        self.set_bulb_current(0)
        self.disable_bulb()
        self.set_indicator_current(0b11)
        self.disable_indicator_led()
        self.set_integration_time(self._integration_time)
        self.set_gain(self._gain)
        self.set_measurement_mode(self._mode)
        if self._sensor_version == SENSORTYPE_AS7262:
            print("AS7262 online!")
        if self._sensor_version== SENSORTYPE_AS7263:
            print("AS7263 online!")

    def get_calibrated_violet(self):
        return self.get_calibrated_value(AS7262_V_CAL)

    def get_calibrated_blue(self):
        return self.get_calibrated_value(AS7262_B_CAL)

    def get_calibrated_green(self):
        return self.get_calibrated_value(AS7262_G_CAL)

    def get_calibrated_yellow(self):
        return self.get_calibrated_value(AS7262_Y_CAL)

    def get_calibrated_orange(self):
        return self.get_calibrated_value(AS7262_O_CAL)

    def get_calibrated_red(self):
        return self.get_calibrated_value(AS7262_R_CAL)

    def get_calibrated_R(self):
        return self.get_calibrated_value(AS7263_R_CAL)

    def get_calibrated_S(self):
        return self.get_calibrated_value(AS7263_S_CAL)

    def get_calibrated_T(self):
        return self.get_calibrated_value(AS7263_T_CAL)

    def get_calibrated_U(self):
        return self.get_calibrated_value(AS7263_U_CAL)

    def get_calibrated_V(self):
        return self.get_calibrated_value(AS7263_V_CAL)

    def get_calibrated_W(self):
        return self.get_calibrated_value(AS7263_W_CAL)

    def get_sensor_type(self):
        if self._sensor_version == SENSORTYPE_AS7262:
            return "AS7262"
        elif self._sensor_version == SENSORTYPE_AS7263:
            return "AS7263"
        else:
            return "Unknown"

    def get_wavelengths(self):
        if self._sensor_version == SENSORTYPE_AS7262:
            return [450, 500, 550, 570, 600, 650]
        else:
            return [610, 680, 730, 760, 810, 860]
