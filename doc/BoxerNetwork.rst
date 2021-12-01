Setting Up Boxer's Network
===========================

Boxer is equipped with an external PC mounted to the top of the robot, referred to as the "Backpack PC".  The robot is
also equipped with an internal PC.  This page explains how to configure the networking for the Backpack PC only.  For
instructions on connecting the internal PC to a network, refer to the
`Otto 100 Documentation <https://help.ottomotors.com>`_


Enabling the Otto App
----------------------

Many of Boxer's features are still accessible via the Otto App.  This requires configuring the base platform to connect
to your wi-fi network.  To do this, connect your laptop to the diagnostic ethernet port on the rear of the robot.
Configure your laptop to have a static IP address on the 10.255.255.0/16 subnet, e.g. 10.255.255.100.

Open a web browser and navigate to http://10.255.255.1:8090.  You will be promted to enter your network credentials.

Refer to `Otto Motors' documentation <https://help.ottomotors.com/latest/commissioning/connecting-a-robot-to-the-network>`_
for more details on configuring the base platform's networking.


Connecting to the Backpack PC
------------------------------

By default, Boxer's wireless is in client mode, looking for the wireless network at the Clearpath factory. In
order to set it up to connect to your own network, you'll have to open up the chassis and connect a network cable to
the PC's ``STATIC`` port. The other end of this cable should be connected to your laptop, and you should give yourself
an IP address in the ``192.168.131.x`` space, such as ``192.168.131.100``. Then, make the connection to Boxer's default
static IP:

.. code-block:: bash

    ssh administrator@192.168.131.1

The default password is ``clearpath``. You should now be logged into Boxer as the administrator user.


Changing the Default Password
-----------------------------

.. Note::

  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your
  robot we recommend changing the password.

To change the password to log into your robot, run the

.. code-block:: bash

  passwd

command.  This will prompt you to enter the current password, followed by the new password twice.  While typing the
passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's SSH service to disallow logging in with a
password and require SSH certificates to log in.  This_ tutorial covers how to configure SSH to disable password-based
login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/

Connecting to Wifi Access Point
--------------------------------

Boxer uses ``netplan`` for configuring its wired and wireless interfaces.  To connect Boxer to your wireless network,
create the file ``/etc/netplan/60-wireless.yaml`` and fill in the following:

.. code-block:: yaml

    network:
      wifis:
        # Replace wlp2s0 with the name of the wireless network device, e.g. wlan0 or wlp3s0 as needed
        # Fill in the SSID and PASSWORD fields as appropriate.  The password may be included as plain-text
        # If you have multiple wireless cards you may include a block for each device.
        # For more options, see https://netplan.io/reference/
        wlp2s0:
          optional: true
          access-points:
            SSID_GOES_HERE:
              password: PASSWORD_GOES_HERE
          dhcp4: true
          dhcp4-overrides:
            send-hostname: true

Once configured, run

.. code-block:: bash

    sudo netplan apply

to bring up your wireless connection.  Running ``ip a`` will show all active connections and their IP addresses.


.. _remote:

Remote ROS Connection
---------------------

To use ROS desktop tools, you'll need your computer to be able to connect to Boxer's ROS master. This can be a
tricky process, but we've tried to make it as simple as possible.

In order for the ROS tools on your computer to talk to Boxer, they need to know two things:

- How to find the ROS master, which is set in the ``ROS_MASTER_URI`` environment variable, and
- How processes on the other computer can find *your computer*, which is the ``ROS_IP`` environment variable.

The suggested pattern is to create a file in your home directory called ``remote-boxer.sh`` with the following
contents:

.. code-block:: bash

    export ROS_MASTER_URI=http://cpr-boxer-0001:11311  # Boxer's hostname
    export ROS_IP=10.25.0.102                          # Your laptop's wireless IP address

If your network doesn't already resolve Boxer's hostname to its wireless IP address, you may need to add
a corresponding line to your computer's ``/etc/hosts`` file:

.. code-block:: bash

    10.25.0.101 cpr-boxer-0001

Then, when you're ready to communicate remotely with Boxer, you can source that script like so, thus defining
those two key environment variables in the present context.

.. code-block:: bash

    source remote-boxer.sh

Now, when you run commands like ``rostopic list``, ``rostopic echo``, ``rosnode list``, and others, the output
you see should reflect the activity on Boxer's ROS master, rather than on your own machine. Once you've
verified the basics (list, echo) from the prompt, try launching some of the standard visual ROS tools:

.. code-block:: bash

    roslaunch boxer_viz view_robot.launch
    rosrun rqt_robot_monitor rqt_robot_monitor
    rosrun rqt_console rqt_console

If there are particular :roswiki:`rqt` widgets you find yourself using a lot, you may find it an advantage to dock them together
and then export this configuration as the default RQT perspective. Then, to bring up your standard GUI, you can simply
run:

.. code-block:: bash

    rqt

.. _bridge:

Reconfiguring the Network Bridge
-------------------------------------

In the event you must modify Boxer's ethernet bridge, you can do so by editing the Netplan configuration file
found at ``/etc/netplan/50-clearpath-bridge.yaml``:

.. code-block:: yaml

    network:
    version: 2
    renderer: networkd
    ethernets:
      # Configure eno1 to communicate with the Otto 100 internal PC via the attachment port
      eno1:
        dhcp4: no
        dhcp6: no
        addresses:
          - 10.252.252.100/16

      # Bridge all the remaining ethernet ports together
      bridge_eth:
        dhcp4: no
        dhcp6: no
        match:
          name: eth*
      bridge_enp:
        dhcp4: no
        dhcp6: no
        match:
          name: enp*
      bridge_enx:
        dhcp4: no
        dhcp6: no
        match:
          name: enx*
    bridges:
      br0:
        # yes, allow DHCP4 connections on the bridge; this allows the backpack PC to accept
        # wired internet connections, e.g. for installing updates
        dhcp4: yes
        dhcp6: no
        interfaces: [bridge_eth, bridge_enp, bridge_enx]
        addresses:
          # Give the bridge the static 192.168.131.1 address for its internal ROS network
          # Any IP-based ROS sensors connected to the backpack should use the 192.168.131.0/24 subnet
          - 192.168.131.1/24

This file will create a bridged interface called ``br0`` that will have a static address of 192.168.131.1, but will
also be able to accept a DHCP lease when connected to a wired router.  By default all network ports named ``en*`` and
``eth*`` are added to the bridge.  This includes all common wired port names, such as:

- ``eth0``
- ``eno1``
- ``enx0123456789ab``
- ``enp3s0``
- etc...

To include/exclude additional ports from the bridge, edit the ``match`` fields, or add additional ``bridge_*`` sections
with their own ``match`` fields, and add those interfaces to the ``interfaces: [bridge_en, bridge_eth]`` line near the
bottom of the file.

We do not recommend changing the static address of the bridge to be anything other than ``192.168.131.1``; changing
this may cause sensors that communicate over ethernet (e.g. lidars, cameras, GPS arrays) from working properly.
