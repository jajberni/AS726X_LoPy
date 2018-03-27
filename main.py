from machine import Pin, I2C
from AS726X import AS726X
import time

sensor_type = "None"

try:
    i2c = I2C(0, I2C.MASTER, pins=('P22', 'P21'))
    sensor = AS726X(i2c=i2c)
    sensor_type = sensor.get_sensor_type()
    time.sleep(1)
    print('Ready to read on wavelengths:')
    print(sensor.get_wavelengths())
except Exception as error:
    print(error)
    pass

while True:
    calibrated_values = [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
    try:
        sensor.take_measurements()
        calibrated_values = sensor.get_calibrated_values()

    except Exception as error:
        print(error)

    print("{sensor_type}:{ch0},{ch1},{ch2},{ch3},{ch4},{ch5}".format(
        sensor_type=sensor_type, ch0=calibrated_values[0],
        ch1=calibrated_values[1], ch2=calibrated_values[2],
        ch3=calibrated_values[3], ch4=calibrated_values[4],
        ch5=calibrated_values[5]))

    time.sleep(1.0)
