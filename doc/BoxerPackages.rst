Configuration & Environment Variables
=========================================

.. image:: graphics/boxer_urdf_banner.png

The boxer_description package is the URDF robot description for Boxer UGV.

.. _Source: https://github.com/boxer-cpr/boxer


Overview
---------

This package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Boxer.  For an example launchfile to use in visualizing this model, see `boxer_viz <http://wiki.ros.org/boxer_viz>`_.


Accessories
------------

Boxer has a suite of optional payloads called accessories. These payloads can be enabled and placed on the robot using environment variables specified at the time the `xacro <http://wiki.ros.org/xacro>`_ is rendered to URDF. Available accessory vars are:

.. raw:: html

    <table><tbody><tr> <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>BOXER_CONTROL_EXTRAS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>BOXER_GPIO</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>??? TODO</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>BOXER_URDF_EXTRAS</tt> </p></td>
      <td><p><tt>empty.urdf</tt> </p></td>
      <td><p>Path to a URDF file with additional modules connected to the robot</p></td>
    </tr>
    </tbody></table>
