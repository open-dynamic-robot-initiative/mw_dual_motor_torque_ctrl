Dual Motor Torque Control
=========================

Firmware for the [Open Robot Actuator Motor Boards](https://github.com/open-dynamic-robot-initiative/open_robot_actuator_hardware) using comunication via CAN.

This firmware allows torque control of two motors at once using communication via CAN.  Use this firmware when you want to connect a TI LaunchPad evaluation board or a BLMC µDriver board via CAN.  If you want to communicate via Ethernet, using a Master Board, use [this firmware](https://github.com/open-dynamic-robot-initiative/udriver_firmware) instead.

This program can be used with the Universal Dual Motor GUI.


Hardware
--------

This project is configured for use with the LAUNCHXL-F28069M microcontroller
board and two DRV8305 booster packs.


Documentation
-------------

For instructions on how to build and use this firmware, please see the
[documentation](https://open-dynamic-robot-initiative.github.io/mw_dual_motor_torque_ctrl).


Structure of this Package
-------------------------

This package is structured as follows:

  * src: Contains all the source code of this project
  * ccs: Contains the Code Composer Studio project files


License
-------

BSD 3-Clause License

Copyright (c) 2016, Max Planck Gesellschaft, New York University
