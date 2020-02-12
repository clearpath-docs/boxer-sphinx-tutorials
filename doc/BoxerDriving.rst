Driving Boxer
===============

Boxer can either drive autonomously, be :doc:`controlled through ROS messages <BoxerSimulation>`, or you
can use the supplied remote control to teleoperate it.


Safety Precautions
----------------------

.. warning::

    Despite its small size, Boxer is capable of reaching high speeds.  Careless driving can cause harm to the operator,
    bystanders, the robot, or other property.  Always remain vigilant, ensure you have a clear line of sight to the
    robot, and operate the robot at safe speeds.


Remote Control
-----------------

Boxer uses a standard Sony Playstation 4 controller for teleoperation.  The left thumb stick is used to control the
robot's speed and direction.  Either L1 or L2 must be held down at all times while driving the robot.  Holding R1
will enable the robot to drive at maximum speed, while holding L1 will restrict the robot's maximum speed.

.. image:: graphics/ps4_controller.jpg
   :alt: Boxer's remote control

.. warning

    While familiarizing yourself with the robot we recommend operating at low speed by holding L1.  Only engage
    full speed (by holding R1) once you are familiar with the robot and there is a safe distance of at least 3m
    around the robot.

If your controller is not pairing correctly with the robot, you can re-pair it by following these steps:

1. Ensure that the controller's batteries have sufficient charge
2. Press and hold the Share & PS buttons on the controller until the pairing light flashes rapidly white
3. Run the following command on Boxer:

.. code-block:: bash

    sudo ds4drv-pair

The controller should now be paired.  Test it by pressing L1 and using the left control stick.


E-Stop Buttons
-------------------

Boxer is equipped with 1 emergency stop button, located on the front the robot.  Pressing this
button will cut power to the motors.  To disengage the emergency
stop, simply twist the button in the direction indicated by the arrows.

.. note::

    Some newer Boxers have an additional emergency stop button located on the back of the robot.  This button operates
    exactly the same, and pressing either button will cut all power to the robot's drive motors.

Whenever you need to perform maintenance on the robot we recommend engaging the emergency-stop if the robot cannot be
fully powered down.


Body Lights
--------------

Boxer includes RGB LED strips around its body.  These lights express system status according to the table below.
In the absence of any of the low-level conditions, they can be commanded from ROS to display indications from autonomy
or other high-level software.

=======================   ========================================================================================================================================================================
Color                     Meaning
=======================   ========================================================================================================================================================================
??? TODO                  ??? TODO
=======================   ========================================================================================================================================================================
