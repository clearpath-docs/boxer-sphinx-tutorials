Keeping Boxer Updated
======================

Boxer is always being improved, both its own software and the many community ROS packages upon which it
depends! You can use the apt package management system to receive new versions all software running on the
platform.


Updating the Base Platform
---------------------------

Otto Motors periodically releases updates for the Otto 100, which can be applied to Boxer's base platform.

Please refer to `Otto Motors' documentation <https://help.ottomotors.com/latest/commissioning/system-deployment/installing-robot-software>`_
for details on applying software updates.

At the time of writing, the latest version of the Otto 100 software supported by the Boxer's Noetic backpack is 2.22.3.


Updating the Backpack PC
-------------------------

The backpack PC can be updated by connecting it to the internet, either using a wired or wireless connection, and
running the following commands:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get upgrade

If you see any errors, please `get in touch`_ and we'll see if we can get you sorted out.

.. _get in touch: https://support.clearpathrobotics.com/hc/en-us/requests/new


.. _scratch:

Starting From Scratch
---------------------

If Boxer's computer has become inoperable, or for any reason you want to restore it to the factory state, you can
reinstall the operating system by following the steps outlined in :doc:`BoxerInstallation`.
