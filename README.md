# firefist
An Arduino-based, punch-activated flamethrower

this project is more or less a clone of this: https://www.hackster.io/Advanced/punch-activated-arm-flamethrowers-real-firebending-95bb80

Using an ATMega328 programmed in Arduino and an MPU6050 (utilizing DMP functionality), a wrist-mounted flamethrower is activated. The ATmega reads gravity-corrected accelleration values from the MPU and runs a simple "punch detection"-algorithm.
The flamethrower is built of a solenoid valve that releases lighter gas through a custom-built nozzle and an "arc lighter". The solenoid is switched by a MOSFET and supplied by a 12V battery pack. The arc lighter is activated by a Photocoupler that shorts the built-in button.

For the Hardware, there is only a Eagle Schematic for documentation purposes. The Circuit has been built on perfboard and not all Parts used in the Schematic have the right footprint!
