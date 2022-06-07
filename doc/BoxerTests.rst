Boxer Tests
============

Boxer robots come preinstalled with a set of test scripts as part of the ``boxer_tests`` ROS package, which can be run to verify robot functionality at the component and system levels. 

If your Boxer does not have the ``boxer_tests`` ROS package installed already, you can manually install it by opening terminal and running:

.. code-block:: bash

  sudo apt-get install ros-noetic-boxer-tests

ROS Tests
----------

The ``ros_tests`` script exposes a set of interactive tests to verify the functionality of core features. These tests run at the ROS-level via ROS topics, and serve as a useful robot-level diagnostic tool for identifying the root cause of problems, or at the very least, narrowing down on where the root cause(s) may be.

Running ROS Tests
------------------

To run ``ros_tests`` on a Boxer robot, open terminal and run:

.. code-block:: bash

  rosrun boxer_tests ros_tests

Upon running ``ros_tests``, a list of available tests will be shown in a menu. From the menu, you can choose individual tests to run, or simply choose the option to automatically run all the tests.

The details of each test are shown below.

**Serial Number Test**

The **Serial Number Test** checks that the robot's serial number is set correctly.

This test checks the ``ROS_ROBOT_SERIAL_NO`` environment variable is set to a valid serial number.

**API Version Test**

The **API Version Test** checks that the robot's API version is set correctly.

This test checks the ``BOXER_API_VERSION`` environment variable is set to a valid API version, and that the robot is publishing data on ROS topics under this API version.

**ROS Bridge Test**

The **ROS Bridge Test** checks that the ROS bridge is working properly, and that the robot's description model is loaded.

This test checks that the list of expected ROS topics from the ROS bridge exist, and that these ROS topics are publishing data at the expected frequencies. The expected ROS topics are published by the robot's base platform, and the ROS bridge "bridges" these ROS topics to the robot's backpack computer, including the base platform's IMU, camera, laser, and battery data.

This test also the robot's model is loaded into the ``robot_description`` ROS parameter.

**E-Stop Test**

The **E-Stop Test** checks that the robot's E-Stop is working properly. 

This test subscribes to the ``/platform/emergency_stop`` ROS topic and checks that when the E-Stop is manually engaged and disengaged by the user, the E-Stop states are correctly reported on the ``/platform/emergency_stop`` ROS topic.

**Rotate Test**

The **Rotate Test** rotates the robot counter clockwise 2 full revolutions and checks that the motors, IMU, and EKF odometry are working properly.

This test:

- Subscribes to the ``/imu/module1/data`` ROS topic to receive angular velocity measurements from the IMU's Gyroscope. These measurements are converted into angular displacement estimations, and the robot will rotate until 2 full revolutions are estimated.
- Subscribes to the ``/odom`` ROS topic to receive angular velocity estimations from the EKF odometry. These measurements are converted into angular displacement estimations, and are output as comparison to the angular displacement estimations from the IMU's Gyroscope.
- Publishes to the ``/cmd_vel`` ROS topic to send drive commands to rotate the robot.
- The user will be asked to verify that the robot rotates 2 full revolutions.

.. note::

  The **Rotate Test** rotates the robot using the IMU's Gyroscope data, which inherently will not be 100% accurate. Therefore, some undershoot/overshoot is to be expected.

**Drive Test**

The **Drive Test** drives the robot forward 1 meter and backward 1 meter, and checks that the motors and EKF odometry are working properly.

This test:

- Subscribes to the ``/odom`` ROS topic to receive linear displacement estimations from the EKF odometry. The robot will drive forward until 1 meter is estimated, then it will drive backward until 1 meter is estimated.
- Publishes to the ``/cmd_vel`` ROS topic to send drive commands to drive the robot.
- The user will be asked to verify that the robot drives forward 1 meter and drives backward 1 meter.

.. note::

  The **Drive Test** drives the robot using the Odometry data, which inherently will not be 100% accurate. Therefore, some undershoot/overshoot is to be expected.