This project implements a Lego Spike robot that follows a light source and reads/translates Morse Code. There are two color sensors on the robot that detect ambient light. If the sensor on the left detects more light, the robot will head toward the left. If the sensor on the right detects more light, the robot will head toward the right.

The project uses the idea of PID controllers to make the robot follow the light more smoothly. The robot will slow down as it approaches the light and stop nicely instead of frantically starting and stopping.

The robot reads a message in Morse code by being provided flashes of light. It translates the message by determining how long the light flashed on and off. After receiving the message, it will display the translated message in English using the light matrix on the robot.
