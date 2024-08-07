# nano-moonlite-clone

A moonlite focuser clone with temperature compensation based on arduino nano 28BYJ-48 stepper and temp probe.
Compatible with INDI and ASCOM based systems.

The driver includes backlash compensation to reduce the effects of the cheap mechanical parts of the 28BYJ-48 stepper. You may need to tweak the value of `BACKLASHSTEPS` in the code of the firmware depending on the quality of your motor.

# Parts

* Arduino nano
* 28BYJ-48 stepper and driver
* DS18B20 temperature probe

Flash the arduino with the firmware, assemble the parts in a project box. Attach to your focuser

# Features

## Saving Focus position on restart

The default position on a cold start is 20,000 unless a previous position was saved.
However, if the saved position is less than 1000 or greater than 30,000 then the position gets reset to 20,000 on start. This can be useful if, for some reason you need to go to a position less than zero. For example, if the camera was manually moved by accident. A workaround for fully remote use, set the focus position to less than 1000, powercycle the device (or PC its connected to) then on start the position will be 20,000


Temp coefficient

Values are +-63. 
