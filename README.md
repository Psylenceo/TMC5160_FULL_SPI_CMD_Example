# TMC5160_FULL_SPI_CMD_Example

##Version 0.1.1 - alpha:
- changed base calc formulas from float to pre-processor defines to cut down memory usage.
- Bug fix - Where trying to calculate TOFF would result in 0. Was due to the first part of the equation resulted in a number too small for the preprocessor to keep. The solution was to start with 100,000 / chop_freq to keep the resulting value within calculation range, then divide by 3,200,000 to give a usable value.

##Version 0.1.0 - alpha:
- Intro to basic registers that need to be set or cleared to get motion in as few lines of code as possible.
