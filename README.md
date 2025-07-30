# Rail Servo

This project controls a stepper-driven linear rail using an ESP32. The ESP32 now starts as a Wi-Fi access point and hosts a web interface that lets a phone control position and speed with sliders and provides homing buttons for three limit switches.

Default Wi-Fi credentials:

- SSID: `rail_servo`
- Password: `123456789`

Connect to the `rail_servo` network (password `123456789`) and open the access point's IP address, typically `http://192.168.4.1/`. You can also use the mDNS name `http://rail_servo.local/`.

## Startup behavior

The travel range and the location of switch&nbsp;1 are configurable in the
firmware. When the ESP32 boots it runs `fullHoming()` to perform a full sequence:

1. It first seeks switch&nbsp;1 and assigns the configured `home1PosCm` value to
   the current position.
2. It then scans toward the positive end to record the positions of switches 2
   and 3.
3. Once switch&nbsp;3 (or the maximum range) is reached, that location becomes
   position zero and the web interface becomes available.

See `servo_rail.ino` for the firmware.
