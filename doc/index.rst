********************************************************************
Documentation of the ODRI Motor Board Firmware for CAN Communication
********************************************************************

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   build_instructions
   can_interface


This documentation describes the **mw_dual_motor_torque_ctrl** firmware for the
`Open Robot Actuator Motor Boards`_ using comunication via CAN.

This firmware allows torque control of two motors at once using communication
via CAN.  Use this firmware when you want to connect a TI LaunchPad evaluation
board or a BLMC µDriver board via CAN.  If you want to communicate via Ethernet,
using a Master Board, use the udriver_firmware_ instead.

This program can be used with the Universal Dual Motor GUI.


Hardware
========

This project is configured for use with the LAUNCHXL-F28069M microcontroller
board and two DRV8305 booster packs.



Structure of this Package
=========================

This package is structured as follows:

  * src: Contains all the source code of this project
  * ccs: Contains the Code Composer Studio project files



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. _Open Robot Actuator Motor Boards: https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware
.. _udriver_firmware: https://github.com/open-dynamic-robot-initiative/udriver_firmware
