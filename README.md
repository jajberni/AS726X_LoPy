# MicroPython library for AS7262 and AS7263 spectral sensors

This is a python library for the AS726X spectral sensors designed to operate with
[PyCom](https://pycom.io/) boards. It is based on [SparkFun's AS726X Arduino Library](https://github.com/sparkfun/SparkFun_AS726X_Arduino_Library)

There is another python library designed for the Raspberry Pi: https://github.com/Shoe-Pi/AS7262_Pi

It has been tested with the following AS726X boards from [SparkFun](https://www.sparkfun.com):
![AS726X](https://github.com/sparkfun/Sparkfun_AS726X_Arduino_Library/blob/master/extras/14351-01.jpg)

[*AS7262 (14347)*](https://www.sparkfun.com/products/14347)

[*AS7263 (14351)*](https://www.sparkfun.com/products/14351)

The AS726X communicates with the PyCom board using the I2C bus.
The default I2C address of the device is 0x49.

**Connecting the board**

The board uses Sparkfun's QWiic connectors. I have used the [Qwiic Cable to Breadboard jumpers](https://www.sparkfun.com/products/14425).
Alternatively you can use normal jumper cables and solder them to the breakout pins on the board.

The default connection is to the 3.3V, GND, SDA (blue) to P22 and SCL (yellow) to P21.
Other pins can be assigned when defining the I2C bus in the main program.

**Testing the library**

Upload the `boot.py`, `main.py`, `lib/AS726X.py` and `Device.py` to your PyCom board.
The demo script will stream the measurements on the serial console at 1Hz.

# Functions

This library provides a class `AS726X` with the functionality of the device.

There are 4 modes of operation:
* Mode 0: Continuous reading of VBGY (7262) / STUV (7263)
* Mode 1: Continuous reading of GYOR (7262) / RTUX (7263)
* Mode 2: Continuous reading of all channels (power-on default)
* Mode 3: One-shot reading of all channels

The default operation (mode 3) requires to take measurements (with or without light)
and then read the calibrated values.

  * **Constructor**

  The constructor takes the i2c port as argument and all the other parameters are optional:

  `sensor = AS726X(i2c=i2c, address=AS726X_ADDR, mode=3, gain=3, integration_time=50`

  * **get_sensor_type()**

  Returns a string with the type of sensor: AS7262 or AS7262

  * **take_measurements()**

  Function to get the breakout to take a single set of measurements (doesn't return anything).


  * **take_measurements_with_bulb()**

  The same as `take_measurements()`, but turns on the white LED on the breakout before measuring and disables it afterwards.

  * **get_wavelengths()**

  Returns an array of the wavelengths for each channel depending on the type of the sensor.

  * **get_calibrated_values()**

  Function to read, process and return stored calibrated values as a list of floats in the order of `get_wavelenghts()`.


  * **set_measurement_mode(mode)**

  Set the measurement mode (0-3) as defined above.


  * **enable_bulb()**

  Turns on the breakout's white LED.  The brightness is controlled by `set_bulb_current()`.  The LED is off by default.


  * **disable_bulb()**

  Turns the breakout's white LED off.


  * **set_bulb_current(current_level)**

  Sets the current provided to the white LED, MUST be passed an integer value of 0, 1, 2 or 3.  More current is brighter, but note that the LED doesn't actually turn on until `enable_main_led()` is used.  Defaults to 12.5 mA (mode 0).

  0 = 12.5 mA

  1 = 25 mA

  2 = 50 mA

  3 = 100 mA


  * **enable_indicator_led()**

  Turns on the breakout's blue indicator LED.  The brightness is controlled by `set_indicator_current()`.  The LED is off by default.


  * **disable_indicator_led()**

  Turns the breakout's indicator LED off.


  * **set_indicator_current(current_level)**

  Sets the current provided to the indicator LED, MUST be passed an integer value of 0, 1, 2 or 3.  More current is brighter, but note that the LED doesn't actually turn on until `enable_indicator_led()` is used.  Defaults to 1 mA (mode 0).

  0 = 1 mA

  1 = 2 mA

  2 = 4 mA

  3 = 8 mA


  * **soft_reset()**

  Soft resets the breakout with a 0.8 second wait for the reset to complete (this time was determined experimentally, anything less seems to cause the I2C bus to timeout).  This should reset all control registers to their default values.


  * **set_gain(gain)**

  Sets the gain of the spectrometer.

  0 = x1   gain

  1 = x3.7 gain

  2 = x16  gain

  3 = x64  gain


  * **set_integration_time(integration_time)**

  Sets the integration time of the readings. Takes values from 0 to 255.
  The integration time will be 2.8ms * [integration value].


  * **get_temperature()**

  Returns the temperature in °C from the sensor using the built in temperature sensor.  
  Pretty inaccurate, according to the documentation: +/-8.5C
