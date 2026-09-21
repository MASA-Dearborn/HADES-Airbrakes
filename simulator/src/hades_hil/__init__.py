"""HADES airbrakes hardware-in-the-loop simulator.

RocketPy 6DOF ascent simulation coupled to the Teensy 4.1 flight computer
over USB serial.  The firmware must be built with -DHIL_MODE
(pio run -e teensy41-hil in the HADES-Airbrakes repo).
"""

__version__ = "0.1.0"
