Dual Motor Torque Control
=========================

Control torque (or actually current) of two motors at once, using
InstaSPIN/SpinTAC.

This program can be used with the Universal Dual Motor GUI.


Hardware
--------

This project is configured for use with the LAUNCHXL-F28069M microcontroller
board and two DRV8305 booster packs.


Requirements and Paths
----------------------

Requires MotorWare as well as the according user configuration files.  They can
be found in the following repositories on git-amd:

  * motorware
  * instaspin-user-config-f2806xM-drv8305 (rename to
	"user_config_f28069m_drv8305" when cloning!)

By default, it is expected that all three packages (this package, motorware and
the config files) are in separate subdirectories of the same root directory.
Example:

  ~/workspace/motorware
  ~/workspace/user_config_f28069m_drv8305
  ~/workspace/dual_motor_torque_ctrl

In case you cannot follow this structure for some reason, you can modify the
path variables $MW_INSTALL_DIR and $USER_CONFIG_DIR	in the project settings.
Please make sure not to commit these changes, though.


Structure of this Package
-------------------------

This package is structured as follows:

  * src: Contains all the source code of this project
  * ccs: Contains the Code Composer Studio project files
