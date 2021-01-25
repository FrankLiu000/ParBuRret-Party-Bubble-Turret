# ParBuRret-Party Bubble Turret

## An Automatic-Toy-Turret-Based-on-8051-Microcontrollers

Some college undergraduates wanted a toy to play with, and they launched this project. It does no harm to anyone, since it’s too weak and shabby.

This thing is called ParBuRret(the abbreviation of Party Bubble Turret), a recursion explantion is ParBuRret Another Reckless Bubble-Urinating Really Rentless Electronic Turret.

I. The project mainly realizes the following functions:

  Build a rotating turret that automatically identifies and "attacks" targets within a range.


II. The project includes the following modules:

1. Servo module: It is composed of two servo motors, which control the rotation in the directions of X axis and Y axis. According to the information provided by the recognition module, the turret rotates to the corresponding position.

2. Recognition module: It is composed of a laser ranging device to detect targets within a certain cone Angle range. It then gives the relative position of the target when it is delivered to the servo module, and decides whether the firing conditions are met.

3. Weapon module: Temporarily we used a bubble blowing machine as "weapon".


III. Hardware used in the project:

1. SCM STC89C52RC ×1

2. PWM modulation chip PCA9685, used to control the servos ×1

3. Lidar VL53L1X, used for ranging and target recognition ×1

4. 360-degree servo ×2

5. Bubble blowing machine that can be triggered by electricity ×1


IV. Software modules required by the project:

1. PCA9685 driver. When the input of a given direction and speed is given, it controls the servos to complete the corresponding action.

2. LiDAR driver. It is required to be able to identify targets within detection range and output the direction and speed in which the turret needs to rotate.

3. Bubble blower driver...


V. Future improvement of the project:

1. The existing target recognition method can be improved a little, such as motion recognition, and the lead amount of shooting can be given through calculation.

2. Weapon modules can be improved.

3. The design of the shell can be improved.
