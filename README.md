# Rail Servo

This project controls a stepper-driven linear rail using an ESP32. The ESP32 hosts a Wi-Fi web interface at `servo_rail.local` that lets a phone control position and speed with sliders and provides homing buttons for three limit switches.

Default Wi-Fi credentials:

- SSID: `rail_servo`
- Password: `123456789`

See `servo_rail.ino` for the firmware.
