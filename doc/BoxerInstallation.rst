Installing Boxer Software
=============================

.. note::

  To get started with the Boxer, make sure you have a :roswiki:`working ROS installation <ROS/Installation>`
  set up on your computer.  The physical Boxer robot comes ROS pre-configured, but if you are working
  on your own computer you may need to follow the instructions on the ROS wiki to get set up.

Add Clearpath Debian Package Repository
------------------------------------------

Before you can install the Boxer packages, you need to configure Ubuntu's APT package manager to
add Clearpath's package server.  You can do this by running the following commands in the terminal:

1. First install the authentication key for the packages.clearpathrobotics.com repository:

.. code-block:: bash

    wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

2. Add the debian sources for the repository:

.. code-block:: bash

    sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'

3. Update your computer's package cache:

.. code-block:: bash

    sudo apt-get update


Installing the Packages
--------------------------

Now that your computer is configured to use Clearpath's deb repository, you can install the Boxer packages needed
for this tutorial by running the following command:

.. code-block :: bash

    sudo apt-get install ros-kinetic-boxer-desktop

If you plan on simulating Boxer in Gazebo, you must also run

.. code-block :: bash

    sudo apt-get install ros-kinetic-boxer-simulator


Installing from Source
---------------------------

.. note::

    As of the time of writing these repositiories are *not* available publicly yet.  They will eventually be
    public, but for now the only option to install Boxer's software is via Ubuntu's package manager.
