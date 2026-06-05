
import logging
import serial
from time import sleep

logger = logging.getLogger(__name__)

class ForceSensor():
    sensor_max: float = 1023

    def __init__(self, port: str):
        self.port = port
        self._connected = False
        self._min_reading: float | None = None

    def is_connected(self):
        return self._connected

    def connect(self):
        if self.is_connected():
            raise ValueError("SENSOR ALREADY CONNECTED")
        self.serial = serial.Serial(self.port, 115200, timeout=1)
        # TODO: eliminate blanket 2s sleep (maybe: add 'hello' from arduino firmware and listen for that)
        sleep(2)
        self._connected = True

    def is_calibrated(self):
        logger.info("dummy is_calibrated() returning True")
        return True

    def calibrate(self):
        logger.info("dummy calibration")
        pass
        
    def read(self):
        self.serial.write("READ\n".encode('utf-8'))
        # TODO: add a timeout, raise on timeout
        while True:
            if self.serial.in_waiting > 0:
                break
        reading = self.serial.readline().decode('utf-8').rstrip()
        scaled = int(reading) * 100 / self.sensor_max
        if self._min_reading is None or scaled < self._min_reading:
            self._min_reading = scaled
        return scaled - self._min_reading

    def disconnect(self):
        self.serial.close()
        self._connected = False

