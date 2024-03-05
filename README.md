# nano-moonlite-clone

A moonlite focuser clone with temperature compensation based on arduino nano 28BYJ-48 stepper and temp probe.
Compatible with INDI and ASCOM based systems.

The driver includes backlash compensation to reduce the effects of the cheap mechanical parts of the 28BYJ-48 stepper. You may need to tweak the value of `BACKLASHSTEPS` in the code of the firmware depending on the quality of your motor.

# Parts

* Arduino nano
* 28BYJ-48 stepper and driver
* DS18B20 temperature probe

Flash the arduino with the firmware, assemble the parts in a project box. Attach to your focuser


