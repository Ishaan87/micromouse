# Micromouse
## File Structure
our code is divided into 5 different files - 
### Config.h 
it contains all the defined constants - such as pins for different sensors, motors, encoders

### Encoders.h
it contains the code for encoders. Now there are some things in this file which I would like to share - 

`volatile long`
tells the compiler that the variable might change and don't try to optimize it! and we used `long` because the int might overflow.
refer to https://docs.arduino.cc/language-reference/en/variables/variable-scope-qualifiers/volatile/ for more

`IRAM_ATTR` - https://esp32.com/viewtopic.php?t=4978

`attachInterrupt()` - https://docs.arduino.cc/language-reference/en/functions/external-interrupts/attachInterrupt/

### Motors.h
contains functions to move motor forward, backward or to stop the motor etc.

### Sensors.h
contains the code for all the sensors - L0X, IMU.
`.rangingTest()` method fires the lidar and stores it into the measure variable (passed by address here). Now the `measure.RangeStatus != 4`, here, the status is '4' if the lidar didn't hit anything (meaning the laser didn't come back)

### Micromouse.ino
Last but definitely not the least, this is the main file which combines all the files together and does all the operations.