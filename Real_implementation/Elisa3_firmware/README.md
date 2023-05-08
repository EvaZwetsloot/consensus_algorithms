# Elisa-3 firmware
This firmware has been adjusted for the implementation of the ETC, PETC and Phase algorithms

## Adjustments
Adjustments include:
- Updated communication protocol (mirf.c)
- Added variables (variables.c)
- Changes to odometry estimation (motors.c)
- Different FSM (main.c)

The framework for this works is the Elisa-3 factory firmware advanced for the ATMega2560 microcontroller. For more information refer to [https://www.gctronic.com/doc/index.php?title=Elisa-3#Advanced_demo](https://www.gctronic.com/doc/index.php?title=Elisa-3#Advanced_demo).
