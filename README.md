# TMC5160_FULL_SPI_CMD_Example

## version 0.1.7 - alpha i stallguard branch
-tweked the code homing positive and negative. as finding and setting the negative end stop position was causing the motor to stop some distance above where the end stop actually was.
- in process of making sure that what is set as the origin actually returns motor to that spot when Xtarget 0 is commanded

## version 0.1.6 - alpha in stallguard branch
- Changed initial homing direction from positive to negative to find my 3d-printers z-axis non energized resting position (all the way down as the weight of the print bed causes the axis to fall with no poer to the motor)
- added second part of homing routine to find the positive end stop

## Version 0.1.5 - alpha in stallguard branch
- currently using a workaround for stallguard that's not as clean or as functionally nice, but is a starting point.
- Began work on setting up sensorless homing
- testing out find pos endstop and save position

## Version 0.1.4 - alpha in stallguard branch
- started testing out and validating stallguard settings
  - Noticed that using stallguard as it should be used and is stated in the datasheet, the tmcstepper library does not have the write functionality available for ramp_stat or event_sg_stop. as of version 4.5. If attempting to use, the only way to clear the stallguard fault is to completely power off the driver. 
- made some new csv data plots. New ones look much cleaner

## Version 0.1.3 - alpha
- Revised the motor performance function due to noisy data during calculations and reporting
- due to the noisey data all the data plots have been deleted

## Version 0.1.2 - alpha
- added some motion calculations to the trinamics excel sheets (none of them are correct, should probably delete them)
- setup a serialplot ini file and added some motion testing plots

## Version 0.1.1 - alpha:
- changed base calc formulas from float to pre-processor defines to cut down memory usage.
- Bug fix - Where trying to calculate TOFF would result in 0. Was due to the first part of the equation resulted in a number too small for the preprocessor to keep. The solution was to start with 100,000 / chop_freq to keep the resulting value within calculation range, then divide by 3,200,000 to give a usable value.

## Version 0.1.0 - alpha:
- Intro to basic registers that need to be set or cleared to get motion in as few lines of code as possible.
