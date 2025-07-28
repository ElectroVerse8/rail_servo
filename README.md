# Rail Servo

This project controls a stepper-driven linear rail using an ESP32. The ESP32 now starts as a Wi-Fi access point and hosts a web interface that lets a phone control position and speed with sliders and provides homing buttons for three limit switches.

Default Wi-Fi credentials:

- SSID: `rail_servo`
- Password: `123456789`

Connect to the `rail_servo` network (password `123456789`) and open the access point's IP address, typically `http://192.168.4.1/`.

See `servo_rail.ino` for the firmware.
