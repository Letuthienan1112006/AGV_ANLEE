# AGV Communication Protocol

## 1. Jetson / real_car_socket.py <-> STM32

Transport: USB Serial
Port: /dev/ttyACM0
Baudrate: 115200

Command from Jetson to STM32:

Format:
steer speed

Examples:
0 0
0 20
-15 30
15 30

Meaning:
- steer: range -30 to 30
- speed: range 0 to 100
- speed = 0 means STOP

STM32 response:

OK steer=X.XX speed=X.XX rpmL=X.XX rpmR=X.XX

Examples:
OK steer=0.00 speed=20.00 rpmL=50.00 rpmR=50.00
OK steer=-15.00 speed=30.00 rpmL=112.50 rpmR=37.50
OK steer=15.00 speed=30.00 rpmL=37.50 rpmR=112.50
OK steer=0.00 speed=0.00

## 2. patrol_robot.py <-> real_car_socket.py

Transport: TCP socket
IP: 127.0.0.1
Port: 54321

Command from patrol_robot.py to real_car_socket.py:

Format:
steer speed

Examples:
0 90
-15 60
15 60

Image from real_car_socket.py to patrol_robot.py:

Raw JSON:
{"Img": "base64_jpeg_string"}

Current version does not use a 4-byte length prefix.

## 3. Safety

- STM32 watchdog stops motors if no command is received.
- real_car_socket.py sends 0 0 when exiting.
- During motor tests, wheels must be lifted off the ground.
- Bench test speed limit: SPEED_MAX = 25.
- Increase speed only after motor direction and PID are verified.
