******************
Connecting via CAN
******************

Hardware
========

The motor board for which this firmware is written supports the CAN 2.0B
standard.
In theory any CAN device following the standard should be able to communicate
with the board.  In practice we are successfully using **"PCAN-PCI Express"**
dual-channel cards for classic CAN by PEAK System.  However, we are having
problems with PEAK's **CAN FD** devices, so we recommend to stick with the
"non-FD" devices for now.  We have not tested with hardware of other
manufacturers, so we cannot say anything about those.


Linux Drivers
=============

For the PEAK devices, SocketCAN drivers are already included in the standard
Linux kernel, so there is no need to manually install any driver.
In the following, it is assumed that SocketCAN is used.


Connection Setup
================

When using SocketCAN, there is an interface for each available channel, called
"can0", "can1", "can2", ...

Each interface has to be configured and enabled in order to be usable.  This
firmware uses 1 Mbit/s and a sample point at 86.7%.  To configure the interface
accordingly, run

.. code-block:: bash

    sudo ip link set can0 type can bitrate 1000000 sample-point 0.867

and to enable it run

.. code-block:: bash

   sudo ip link set up can0

This has to be done for each interface that you want to use (replace "can0"
accordingly).


Determine Names of Interfaces
-----------------------------

The names are assigned automatically to the interfaces and the order may not
seem logical from outside.  So you first need to find out which physical port
"can0", "can1", etc. correspond to.

There may be nicer ways to do this but the following precedure works well for
us:

1. Disconnect any devices from the CAN ports.
2. Configure and enable all interfaces as described above.
3. Open a terminal and run `watch netstat -i`.  It should show an output like
   this::

       Kernel Interface table
       Iface   MTU Met   RX-OK RX-ERR RX-DRP RX-OVR    TX-OK TX-ERR TX-DRP TX-OVR Flg
       can0         16 0         0      0      0 0             0      0      0      0 ORU
       can1         16 0         0      0      0 0             0      0      0      0 ORU
       can2         16 0         0      0      0 0             0      0      0      0 ORU
       ...

4. Use a motor board that has been flashed (or any other CAN device that sends
   messages), power it on and connect it to the first CAN port and watch the
   output in the terminal.  For one of the listed interfaces, the RX-OK value
   should start increasing.  This is the interface you are currently connected
   to.
5. Disconnect the board and connect it to the next port, repeat until all ports
   are identified.  You may want to put labels on them.

Note: As long as the hardware configuration of the computer is not changed, the
interface to port mapping should be fixed.  However, when adding more CAN cards
or removing some of the existing ones, the order of the others may change.  In
this case, the identification procedure needs to be repeated.



Testing the Connection
======================

For a first basic test if the CAN communication is working, the console
applications ``candump`` and ``cansend`` can be used.  On Ubuntu, they can be
installed via the ``can-utils`` package.

In the following it is assumed that interface "can0" is used.  Adjust the
commands accordingly when using a different interface.

1. Connect the motor board via CAN to the computer and power it on.
2. Open a terminal and run

   .. code-block:: bash

      candump -e can0

   It should display a message with ID 010 once every second (the status message
   that is send by the board).  If you don't see this message, something is
   wrong!

3. If you see the status messages, you can proceed by sending the commands to
   enable the motors.  Open a separate terminal (keep the ``candump`` running!)
   and execute the following commands in this order:

   .. code-block:: bash

      cansend can0 005#0000000000000000  # set target current to zero
      cansend can0 000#000000000000001E  # disable CAN receive timeout
      cansend can0 000#0000000100000001  # enable the system
      cansend can0 000#0000000100000014  # enable sending measurements
      cansend can0 000#0000000100000002  # enable motor 1
      cansend can0 000#0000000100000003  # enable motor 2

   After the the command to enable sending measurements, the output of
   ``candump`` should start sending messages with IDs 020, 030, 040 and 050 at
   high frequency.

   When enabling a motor the corresponding motor should jitter a bit and then be
   held in place for a few seconds (assuming there actually is a motor
   connected).  This is an initial calibration procedure that is automatically
   performed when the motor is enabled for the first time after power up, see
   :doc:`motor_alignment`.

If everything behaves as described, this means that the CAN communication is
generally working.  As an additional test, you may want to check if the
communication rate is stable.  For this, keep the board on after step 3 and run
the following command:

.. code-block:: bash

   candump -t d can0,040:FFF

This will print only messages with ID 040.  The first value in each row is the
time passed between the current message and the previous one.  This value should
be close to 0.001 (= 1 ms) with only little deviation.  If you see larger
deviations here, this means some messages are delayed, indicating an instable
connection.  Whether this is a problem depends on the application but in general
delays mean that the controller will be less stable.
