This folder is for testing a ROS hardware driver based
on the Pololu I2C driver for the Romi 32u4 driver board.

TODO
* Raspberry Pi
** Twist - ROS subscriber for to relay motor speeds to Romi
** Encoder - ROS publisher to publish encoder positions
** ODOM - ROS publisher to publish odometry
** TF - ROS publisher to publish TF
** Modify driver for new I2C transfer fields
* Arduino 32u4
** Copy over existing Arduino for Romi
** Copy Odom code from Ching-Chings work
** Add odom variables to the I2C transfer format

Dependent Items:
* Arduino 32u4 Code is multirobotics/robots/Romi-All/Arduino/romipi_rosserial_twist_odom
* Raspberry Pi ROS code is multirobotics/robots/Romi-All/ROS/romipi/romipi_i2c
