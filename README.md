# Rail Servo

This project controls a stepper-driven linear rail using an ESP32. The ESP32 now starts as a Wi-Fi access point and hosts a web interface that lets a phone control position and speed with sliders and provides homing buttons for three limit switches.

Default Wi-Fi credentials:

- SSID: `rail_servo`
- Password: `123456789`

Connect to the `rail_servo` network (password `123456789`) and open the access point's IP address, typically `http://192.168.4.1/`. You can also use the mDNS name `http://rail_servo.local/`.

## Startup behavior

When the ESP32 boots it performs a full homing sequence:

1. It homes toward switch 1 to establish the negative end at −15&nbsp;cm.
2. It then moves to find switch 2 and switch 3, storing their positions.
3. After all switches are detected the current position is reset to zero and the web interface becomes available.

See `servo_rail.ino` for the firmware.
