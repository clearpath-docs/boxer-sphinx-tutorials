Installing Boxer's OS
=============================

.. note::

  To get started with the Boxer, make sure you have a :roswiki:`working ROS installation <ROS/Installation>`
  set up on your computer.  The physical Boxer robot comes ROS pre-configured, but if you are working
  on your own computer you may need to follow the instructions on the ROS wiki to get set up.

Using Clearpath's ISO
----------------------

.. note::

    At the time of writing Boxer support has not been added to Clearpath's Noetic-Focal installation ISO.  Work on
    this will be done ASAP, and this notice will be removed when the ISO as been updated to support Boxer.

The easiest way to set up the OS on Boxer's backpack PC is to use Clearpath's installation ISO for Ubuntu 20.04 with
ROS Noetic.  This ISO supports installing all of Boxer's packages during the OS installation, and will configure
the computer's networking to communicate with the base platform.

To use Clearpath's ISO, first `download the latest version of the image <https://packages.clearpathrobotics.com/stable/images/latest/noetic-focal/amd64/>`_.
Use a tool like Unetbootin, Balena Etcher, or Rufus to write the ISO to a USB drive.

.. note::

    If you use Unetbootin to create a bootable USB drive you will see a warning saying ``Unetbootin media detected``
    when you boot from the drive.  Ubuntu adds this warning automatically.  In our experience there have not been any
    issues installing the OS using media created with Unetbootin, but if you have concerns use a different tool like
    Etcher or Rufus instead.

The OS installation will require an internet connection via ethernet.  Connect an ethernet cable connected to your
router to ``eno1`` -- note that this is the same port normally used for communicating with the base platform.  If
``eno1`` (sometimes labelled LAN1) is connected to the base platform, disconnect the base platform for now.  Once the OS
has installed you can reconnect the base platform to ``eno1``.  You will also need to connect a monitor and keyboard to
the backpack PC.  We recommend connecting the Boxer to its charger during the installation process to ensure it
doesn't lose power.

Insert the USB drive you created and boot from it.  You may need to edit the BIOS or UEFI to change the boot device
priority.  Normally you can access this menu by pressing the Delete key immediately after the PC powers-on.

During the OS installation you will be asked to select which robot you are installing the OS on.  Select ``boxer`` from
the list of options.

You will then be asked to enter the robot's serial number.  This is stamped on the back of the robot, and will be of
the form ``A31_`` followed by a series of numbers.  It is important that this serial number be entered correctly.
Otherwise you may have errors communicating with the ROS2 API published by the base platform.

The next prompt will ask you for the robot's hostname.  This can be anything you want, but should be unique to
each robot.

Finally you may be prompted to choose which drive to install the OS to.  Make sure to select the PC's internal drive,
and not the USB drive you are installing from.

After confirming the drive and affirming that you understand that installing the OS will wipe all existing data from
the drive, the OS installation will proceed automatically.  The PC will power-off when the installation finishes.

Once the OS is installed, remove the ethernet cable from ``eno1`` and reconnect the base platform to this port.

Proceed to :ref:`backpack-setup`.


Setting Up Manually
--------------------

.. note::

    Follow these steps if you are using a computer that does not use the ``x86_64`` (aka ``x64`` or ``amd64``)
    architecture (e.g. an Nvidia Jetson computer), or if you cannot/do not wish to use Clearpath's official
    OS installation image.

First, you will need to edit your ``/etc/apt/sources.list`` and/or create the necessary files in
``/etc/apt/sources.list.d`` to add the ROS1 Noetic, ROS2 Foxy, and Clearpath package sources.  Follow the instructions
on the following pages:

* `ROS1 Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_

* `ROS2 Foxy <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>`_

* `Clearpath Packages <http://packages.clearpathrobotics.com>`_

You will also need to configure the computer's networking to assign one port (``eno1`` normally) to be the connection
to the base platform.  Refer to :ref:`bridge` for details on configuring the ethernet bridge.

Once the package sources have been added and the bridge configured, you can install the Boxer packages either
by installing them with the ``apt`` command or from source.  See :doc:`BoxerPackages`
for details on installing the Boxer packages.

Proceed to :ref:`backpack-setup`.


.. _backpack-setup:

Configuring the Backpack
--------------------------

If you are installing the packages on Boxer's backpack PC, you will need to set up ``/etc/ros/setup.bash``
too:

.. code-block:: bash

    # Mark location of self so that robot_upstart knows where to find the setup file.
    export ROBOT_SETUP=/etc/ros/setup.bash

    # Setup robot upstart jobs to use the IP from the network bridge.
    # export ROBOT_NETWORK=br0

    # Insert extra platform-level environment variables here. The six hashes below are a marker
    # for scripts to insert to this file.
    ######

    # Pass through to the main ROS workspace of the system.
    source /opt/ros/noetic/setup.bash

    # Source your catkin workspace
    # Make sure to use the complete path, and avoid using envars like $HOME
    # Omit this if you do not have a workspace
    source /home/administrator/catkin_ws/devel/setup.bash

    # Set the ROS_ROBOT_SERIAL_NO and BOXER_API_VERSION envars
    # this is required for the Boxer to operate correctly
    # The serial number must match the one stamped on the back of the robot
    export ROS_ROBOT_SERIAL_NO=A31_0123456789
    export BOXER_API_VERSION=v1_3

    # Any additional environment variables that depend on your workspace should be exported here
    # e.g.
    #export BOXER_URDF_EXTRAS=/path/to/boxer_customizations.urdf.xacro

The ``BOXER_API_VERSION`` environment variable must be set on the robot's backpack PC.  This version is determined by
the version of the ROS2 API running on the base platform.

At the time of writing the latest version of the Otto software uses ``v1_3``

The ``ROS_ROBOT_SERIAL_NO`` environment variable must match the serial number stamped on the back of the robot.
The serial number is case-sensitive, and will begin with ``A31_``, followed by a series of numbers.

Finally, after you have configured ``/etc/ros/setup.bash`` you can run the following to create the ROS systemd jobs
that will start ROS automatically when the backpack PC start up:

.. code-block:: bash

    source /etc/ros/setup.bash
    rosrun boxer_bringup install
    sudo systemctl daemon-reload
    sudo systemctl start ros
    sudo systemctl start ros-bridge
