Installing from Source
---------------------------

.. note::

    As of the time of writing these repositiories are *not* available publicly yet.  They will eventually be
    public, but for now the only option to install Boxer's software is via Ubuntu's package manager.

The source code for the base Boxer packages is available on GitHub_.  We recommend installing the software through
**apt**, as described above.  But if you want to modify Boxer's behaviour, or simply learn more about how the robot
is configured, you can check out and build the packages yourself.  These instructions assume you are at least somewhat
familiar with building ROS packages using the :roswiki:`Catkin build system <catkin/conceptual_overview>`.

.. _GitHub: https://github.com/boxer-cpr/

First create a workspace directory and initialize it:

.. code-block:: bash

    mkdir ~/boxer_ws
    cd ~/boxer_ws
    mkdir src
    catkin_init_workspace src

Next clone the Boxer repositories using git:

.. code-block:: bash

    cd ~/boxer_ws/src
    git clone https://github.com/boxer-cpr/boxer.git
    git clone https://github.com/boxer-cpr/boxer_simulator.git
    git clone https://github.com/boxer-cpr/boxer_desktop.git

Note that there are three separate git repositories being cloned:

+----------------------+----------------------+---------------------------------------------------------------------+
| Git repository       | ROS Packages         | Description                                                         |
+======================+======================+=====================================================================+
| ``boxer``            | * boxer_control      | Common packages for the Boxer platform, including messages and      |
|                      | * boxer_description  | robot description.  These packages are relevant to all workspaces,  |
|                      | * boxer_msgs         | including simulation, desktop, or use on the robot itself.          |
+----------------------+----------------------+---------------------------------------------------------------------+
| ``boxer_simulator``  | * boxer_gazebo       | Packages essential for running boxer simulations.  Requires the     |
|                      | * boxer_simulator    | packages from the ``boxer`` repository.                             |
+----------------------+----------------------+---------------------------------------------------------------------+
| ``boxer_desktop``    | * boxer_desktop      | Packages for controlling & monitoring the physical robot and/or     |
|                      | * boxer_viz          | simulation.  Requires the packages from the ``boxer`` repository    |
+----------------------+----------------------+---------------------------------------------------------------------+

Now install additional ROS dependencies:

.. code-block:: bash

    cd ~/boxer_ws
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

Finally build the workspace:

.. code-block:: bash

    cd ~/boxer_ws
    catkin_make

You can now source your workspace's in order to make use of the packages you just built:

.. code-block:: bash

    cd ~/boxer_ws
    source devel/setup.bash

To test that everything worked, try running the Boxer simulation that we'll be using in the next portion of this
tutorial:

.. code-block:: bash

    roslaunch boxer_gazebo boxer_world.launch
