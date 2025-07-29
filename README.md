Traceback (most recent call last):
  File "/home/pi/rov/myenv/mission.py", line 2, in <module>
    import board
  File "/home/pi/rov/myenv/lib/python3.11/site-packages/board.py", line 48, in <module>
    from adafruit_blinka.board.raspberrypi.raspi_4b import *
  File "/home/pi/rov/myenv/lib/python3.11/site-packages/adafruit_blinka/board/raspberrypi/raspi_4b.py", line 6, in <module>
    from adafruit_blinka.microcontroller.bcm2711 import pin
  File "/home/pi/rov/myenv/lib/python3.11/site-packages/adafruit_blinka/microcontroller/bcm2711/pin.py", line 7, in <module>
    from adafruit_blinka.microcontroller.generic_linux.rpi_gpio_pin import Pin
  File "/home/pi/rov/myenv/lib/python3.11/site-packages/adafruit_blinka/microcontroller/generic_linux/rpi_gpio_pin.py", line 6, in <module>
    from RPi import GPIO
ModuleNotFoundError: No module named 'RPi'


