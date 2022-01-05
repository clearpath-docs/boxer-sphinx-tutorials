Boxer ROS Packages
===================

.. image:: graphics/boxer_urdf_banner.png


Boxer's Noetic packages are split into 5 repositories on Github:

- `Boxer <https://github.com/boxer-cpr/boxer>`_ -- the core description and control packages, needed for physical and simulated robots
- `Boxer Robot <https://github.com/boxer-cpr/boxer_robot>`_ -- core launch files and services needed for operating a physical Boxer
- `Boxer Simulation <https://github.com/boxer-cpr/boxer_simulation>`_ -- Gazebo simulation packages for Boxer
- `Boxer Desktop <https://github.com/boxer-cpr/boxer_desktop>`_ -- desktop visualization packages for Boxer
- `Boxer Manipulation <https://github.com/boxer-cpr/boxer_manipulation>`_ -- optional support for robotic arms and grippers, including Kinova, Kuka, Robotiq, and UR


Installation from Debian Packages
----------------------------------

.. note::

    At the time of writing, not all of the Boxer packages have been released as .deb packages yet.  These are
    queued, and will be released ASAP.  In the meantime, all the packages can be installed from source using the
    method outlined below.

The preferred way to install Boxer's ROS packages is using precompiled Debian packages.  These packages are available
for Ubuntu 20.04.

To install these packages on your computer, ensure that you have added the ROS1 Noetic and ROS2 Foxy sources to your
``apt`` configuration, as well as Clearpath's package server:

* `ROS1 Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_

* `ROS2 Foxy <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>`_

* `Clearpath Packages <http://packages.clearpathrobotics.com>`_

Note that Foxy is only needed if you plan on installing the ``boxer_robot`` packages; because of the Otto 100's ROS2
API the Foxy version of ``ros_bridge`` is required to operate the physical robot.

Once you have added all of the new apt sources, simply run

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install ros-noetic-boxer-description ros-noetic-boxer-desktop ros-noetic-boxer-simulation ros-noetic-boxer-robot


Note that the ``boxer_manipulation`` packages are not available as .deb packages, because they have additional
dependencies that can only be installed from source.  See below for details on building packages from source, and
refer to `boxer_manipulation on Github <https://github.com/boxer-cpr/boxer_manipulation>`_ for more details on using
these packages.


Installation from Source
-------------------------

Boxer packages can be compiled and installed from source if desired.  Boxer uses the normal ``catkin`` build tools.

To compile the Boxer packages, first create a catkin workspace if you do not already have one:

.. code-block:: bash

    source /opt/ros/noetic/setup.bash
    mkdir -p $HOME/catkin_ws/src
    cd $HOME/catkin_ws
    catkin_init_workspace src

Then clone the necessary packages from Github.  Depending on your needs, not all packages will be necessary.

.. code-block:: bash

    cd $HOME/catkin_ws/src
    # you'll likely always need the core boxer description and msg packages:
    git clone https://github.com/boxer-cpr/boxer.git

    # desktop packages are only needed if you'll be using Rviz to monitor or
    # control a real or simulated robot
    git clone https://github.com/boxer-cpr/boxer_desktop.git

    # simulation packages are only needed if you'll be using the Gazebo simulations
    git clone https://github.com/boxer-cpr/boxer_simulation.git

    # robot packages are only needed on the backpack PC connected to the base platform
    git clone https://github.com/boxer-cpr/boxer_robot.git

    # manipulation packages are only needed if your robot is equipped with a robotic arm or gripper
    git clone https://github.com/boxer-cpr/boxer_manipulation.git

Install the necessary dependencies using ``rosdep``

.. code-block:: bash

    cd $HOME/catkin_ws
    rosdep install --from-paths src --ignore-src --rosdistro=noetic -r

.. note::

    If you included the ``boxer_manipulation`` repository you will need to install additional dependencies from
    source, e.g. the ``ros_kortex`` driver for Kinova arms, or the ``universal_robots_ros_driver`` for UR arms.  Refer
    to the ``boxer_manipulation`` `github page <https://github.com/boxer-cpr/boxer_manipulation>`_ for more information.

Finally, build the workspace:

.. code-block:: bash

    cd $HOME/catkin_ws
    catkin_make

If you are setting up a Boxer backpack PC, see :ref:`backpack-setup` for additional steps needed.


Boxer Environent Variables
---------------------------

Like all Clearpath robots, Boxer supports additional customization through the use of environment variables.

The following table lists the available environment variables and their effect on the robot.

.. raw:: html

    <table>
      <tbody>
        <tr>
          <td><p><strong>Variable</strong></p></td>
          <td><p><strong>Default</strong></p></td>
          <td><p><strong>Description</strong></p></td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>ROS_ROBOT_SERIAL_NO</tt> </p></td>
          <td><i>undefined</i></td>
          <td><p>The Boxer's serial number.  This should be of the form <tt>A31_0123456789</tt> and must match the serial number stamped on the robot</p></td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>BOXER_API_VERSION</tt> </p></td>
          <td><tt>v1_1</tt></td>
          <td><p>The version of the Otto SDK running on the base platform.</p></td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>BOXER_URDF_EXTRAS</tt> </p></td>
          <td><tt>empty.urdf.xacro</tt></td>
          <td><p>Optional path to an additional URDF file to be added to the robot's description.  Commonly used to add additional joints and links to add additional payloads, e.g. arms, additional sensors</p></td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>BOXER_CONTROL_EXTRAS</tt> </p></td>
          <td><tt>empty.yaml</tt></td>
          <td><p>Optional path to a configuration file to override any of the Boxer's control parameters.  Commonly used to change controller button mappings, EKF parameters, etc...</p></td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>BOXER_PC</tt> </p></td>
          <td><tt>1</tt></td>
          <td><p>If <tt>1</tt> the URDF will include a model of the backpack PC.  If <tt>t</tt> the backpack PC is omitted from the URDF.  The shape of the mode is determined by the <tt>BOXER_PC_MODEL</tt> variable</p></td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>BOXER_PC_MODEL</tt> </p></td>
          <td><tt>evs-2000</tt></td>
          <td>
            <p>Customizes the shape of the backpack PC added to the URDF. Must be one of the following:
            <ul>
              <li><tt>evs-2000</tt> (default): the Vecow EVS-2000 series computer (or equivalent case)</li>
              <li><tt>ecx-1000</tt>: the Vecow ECX-1000 series computer (or equivalent case)</li>
              <li><tt>mini-itx</tt>: a common mini-ITX mini desktop enclosure</li>
            </ul>
            </p>
          </td>
        </tr>
        <tr>
          <td><span class="anchor" id="line-11"></span><p><tt>BOXER_GPIO</tt> </p></td>
          <td><tt>0</tt></td>
          <td><p>If <tt>1</tt> the GPIO pins on the PC can be controlled via a ROS node.  <strong>Not yet supported</strong></td>
        </tr>
      </tbody>
    </table>
