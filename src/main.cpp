/******************************************************
    This example code shows all the options for a TMC5160 to be controlled from only SPI.
    The code will be broken up just a little bit to show the various registers that need to
    be set to get each function running.

    Currently this example works best on teemuatluts TMC stepper library version 4.5 and newer.
    
    Versions 4.3 and olderare missing some of the functions do not work or values are sent to wrong registers.
    
    For version details see teemuatluts github page for info..
    https://github.com/teemuatlut/TMCStepper
    
    Code written by: Psylenceo    
    Also Authour tends to use {} as code folding points.

    I have included the option to use an arduino uno or nano, but the code may become larger than the 328P on the nano or uno can handle.
    If that is the case use a more powerful processor, comment out some of the dialog options, or if I can figure it out I'll block off sections
    with a chip identifier if statement so that at compile time it will only compile what is needed and can fit on the 328P.
 *  *********************************************************************/
#include <Arduino.h>                          //Include the arduino header for use within platformIO
#include <TMCStepper.h>                       //header for tmcstepper library
#include <TMCStepper_UTILITY.h>               //header for thcstepper utility library

#define TMC5160_FULL_SPI_CMD_Example_VERSION 0x000101  //v0.1.1 - alpha

#define pi 3.1415926535                       //Pi is used in calculating the drivers initial pwm gradient for autotuning the motor

/****************************************************
   This code uses the arduino hardware SPI, but software SPI
   can be used, I just have not set it up.
 *****************************************************/
/* If using arduino mega*/
#define sck      52                                    //SPI clock
#define mosi     51                                    //master transmit out slave receive input
#define miso     50                                    //master receive input slave transmit out
#define ss       53                                    //chip select

/*if using arduino uno, or nano*/
/*#define sck      13                                  //SPI clock
  #define mosi     11                                  //master transmit out slave receive input
  #define miso     12                                  //master receive input slave transmit out
  #define ss       10                                  //chip select
*/

#define drv_en 7                                       //pin 7 of the arduino will be used for driver enable and disable can be a pin of your choice

#define sense_resistor .075                            //Change this to the value of your sense resistor

#define supply_voltage 24                              //change this to the voltage you are trying to run the driver chip and motor at

/********************************************************
   Example motor -> Kysan 1040118 17HD-B8X300-0.4A  -> http://kysanelectronics.com/Products/datasheet_display.php?recordID=8585
   Operating voltage-> 12 volts
   operating current -> .4A   (400mA)
   coil resistance ->  30 ohms
   coil inductance ->  37 mH
   Holding torque -> 26 Ncm (260 mNm)
   Rotor torque -> 35 gcm^2
   Degrees ->1.8
   All of that info was from the data sheet and some of it is shown on the product page
 **********************************************************/
#define motor_voltage 12                              //Motor operating voltage shown on datasheet. Change tp match your motor.
#define motor_milliamps 400                           //Milliamps specified on motor datasheet. Change to match your motor.
#define motor_resistance 30                           //Motor coil resistance from datasheet. Change to match your motor.
#define motor_hold_torque 260                         //Motor holding torque from datasheet. Change to match your motor. May need to calculate
#define motor_step_degrees 1.8                        //angle of rotation per step of motor
#define motor_us_counts (360/motor_step_degrees)       //number of full steps per full rotation of motor. 360 / degrees = full step count

/****************************************************
   Now we need to define some of the base setting that
   we'll be using with the driver chip.

   This example will be using the drivers internal clock which runs at 12MHz.

   Also in the drivers data sheet, in the stealth chop section
   it talks about pwm freq, clock frequency, and good ranges of operation.
   to try and stay within 20kHz-50kHz range, with the the optimum pwm freq
   being in the green section for a given clock frequency and selected PWM frequency.

   Since this example is using the internall 12MHz clock and
   we want the widest adjustment range, we will be using
   pwm_freq = 2, for starting frequency of 35.1kHz.
 ***********************************************************/
#define drv_clock 12000000                          //using internal clock, tie the clk pin low. If using a different freq, change this value to match
#define drv_chop_freq 35100                         //drivers chop frequency set by pwm_freq and based on clk frequency. Change to match.
#define drv_decay_percent .7                        //percentage (as a decimal) of chopper standstill cycle time for lower power dissipation and upper frequency limit
#define drv_microstep_res 256                       //number of micro steps per full step. 

/***************************************************************
   Motor count calculations, these are mostly if you are using this code to
   test on a stepper with a lead screw, belt, or other linear motion.

   These calculations will specifically be in the xtarget mm command or mm per second function
   since the program will need to know how many counts on your motor actually move a reference point 1mm.
   Think of a stepper motor with a lead screw being used on the z axis of a 3d printer. You want to
   be able to tell the axis to move # mm's and it actually move that many mm's.

   To get these numbers is straight forward. Be in counts position command mode (i'll think of an easier name)
   and make sure you have a "zero" reference point. Then measure (preferably with calipers or depth micrometer) the distance from
   your "zero" point to a know flat surface. 
   
   Command the motor to move 1000 counts and hold position. Remeasure from your "zero" spot to the known flat surface. 
   Then subract your original position from your new position and divide that value by 1000. This gets you how much your "zero" 
   reference point will move per micro step count. 
   
   Example, the Kysan motor I used as an example earlier is the original lead screw motor for makergear m2 printers.
   Using the procedure described above, each micro step of the motor will move a makergear m2 print bed spider arm
   .00015703125mm or .157 micrometers per micro step.
 *****************************************************************/
#define base_reference_distance 0
#define distance_thousand_steps 0
#define motor_mm_per_microstep .00015703125           //Change this value to match your motor!!!!

/************************************************************
   Now we need to calculate some important values for inital register settings in the driver. If you want to adjust any
   of the register settings after initialization. You can change them below in the setup section.

   First we calculate the nominal amperage of the motor, this is important if we are using a voltage other than what the motor
   is rated for:
   For example 12 volt @ .4 amps = 4.8 watts if we use 24 volts to power the motor
   then 4.8 watts / 24 volts = .2 amps.

   This is then used to set the Irun and Ihold registers along with tuning of the chopper modes.

   Next we calculate the back emf constant of the motor which is used to calculate the
   initial gradient for stealth chop to ramp up at.

   Then we calculate the PWM gradient, which will give the autotune a starting point.
   We also need to calculate the initial PWM offset.
   Finally we calculate the drivers PWM off time.
 ***********************************************************/

#define nominal_amps (((motor_milliamps * motor_voltage) / supply_voltage) * 1.0)                                //calculate the voltage based curent
#define cbemf (motor_hold_torque / (2 * nominal_amps))                                                           //calc the back emf constant of the motor
#define microsteps_per_rev ((motor_us_counts *  drv_microstep_res))
#define drv_pwm_grad ((cbemf * 2 * pi * ((drv_clock * 1.46) / (supply_voltage * microsteps_per_rev))))           //calculate the pwm gradient
#define drv_pwm_ofs ((374 * motor_resistance * (nominal_amps / 1000)) / supply_voltage)                          //calculate teh pwm offest
#define driv_toff ((((100000 / drv_chop_freq) * drv_decay_percent * .5) * drv_clock - 12) / 3200000)             //calculate the toff of the motor, currently does not calculate value

/***********************************************************
   Using the example motor, this gives us results of:

   Nominal amps -> 200mA
   cbemf -> .65
   PWM gradient -> 58.2               (the register will round down to 58)
   PWM starting offset -> 93.5        (this one may round up or down)
   Toff -> 3.36                       (so toff should be set to 3 or 4, may need some testing)

   Now that we have our initializers calculated we need to tell the arduino what driver we are using and and give the register points
   a prefix. This is needed, but is useful when multiple motors are used with a single CPU.
 **********************************************************/

TMC5160Stepper driver = TMC5160Stepper(ss, sense_resistor); //we are also telling the libray what pin is chip select and what the sense resistor value is.

/**********************************************************
   Prototypes of functions.
   This makes this area much smaller, because it lets us put
   the function and everything inside the function below the main loop
 ***********************************************************/
void base_calc_values(void);                     //prototype, but this will show what the calculations above result in as initial values
void read_motor_performance(void);               //prototype function to read only the specific register address from the TMC5160
void Ramp_settings(int units, double vstart, double a1, double v1, double amax, double vmax, double dmax, double d1, double vstop);           //prototype function to select and enter the units of your choice instead of just counts

double time,                                      //variable to keep track of the time between motor performance samples
      position,                                  //variable to keep track of the position between motor performance samples
      velocity,                                  //variable to keep track of the velocity between motor performance samples
      old_velocity,                              //testing out better accel and jerk measurement
      dT,                                        //variable to keep track of the time difference between the arduino and the drivers time between motor performance samples   
      dx,                                        //variable to keep track of the position difference between arduino and driver between motor performance samples
      accel,                                     //variable for calculated acceleration 
      old_accel,                                 //testing out better accel and jerk measurement
      jerk,                                      //variable for calculated jerk
      tsteps,                                    //time between steps
      load,                                      //variable to keep track of the mechanical load between motor performance samples 
      coil_a,                                    //variable to keep track of the coil a current between motor performance samples
      coil_b;                                    //variable to keep track of the coil a current between motor performance samples

int homing_calibrate[10],                        //homing points
    homing_count,
    pos_home,
    neg_home;

void setup() {
  /* start up uart config as PC interface */{
    Serial.begin(115200);                   //serial com at 115200 baud
    while (!Serial);                        //wait for the arduino to detect an open com port
    //Serial.println("Start...");             //com port is open, send 1st mesage
    //Serial.println("");                     //add a new line to separate information
    //base_calc_values();                     //readout the defined calculations
    delay(5000);                           //wait 10 seconds so user can read values
  }

  /* start up SPI, configure SPI, and axis IO interfacing*/{
    SPI.begin();                            //start spi
    pinMode(sck, OUTPUT);                   //set sck pin as output for spi clock
    pinMode(ss, OUTPUT);                    //set ss pin as an out put for chip select
    pinMode(drv_en, OUTPUT);                //set drv enable pin as out put
  }

  driver.toff(0);                           //clear status bits in driver
  digitalWrite(drv_en, LOW);                //enable the driver so that we can send the initial register values

  /*Initial settings for basic SPI command stepper drive no other functions enabled*/ {
    driver.begin();                         // start the tmc library

    /* base GCONF settings for bare stepper driver operation*/    {
      driver.recalibrate(0);                //do not recalibrate the z axis
      driver.faststandstill(0);             //fast stand still at 65ms
      driver.en_pwm_mode(0);                //no silent step
      driver.multistep_filt(0);             //normal multistep filtering
      driver.shaft(0);                      //motor direction cw
      driver.small_hysteresis(0);           //step hysteresis set 1/16
      driver.stop_enable(0);                //no stop motion inputs
      driver.direct_mode(0);                //normal driver operation
    }

    /* Set operation current limits */
    driver.rms_current(nominal_amps, 1);    //set Irun and Ihold for the drive

    /* short circuit monitoring */    {
      driver.diss2vs(0);                    //driver monitors for short to supply
      driver.s2vs_level(6);                 //lower values set drive to be very sensitive to low side voltage swings
      driver.diss2g(0);                     //driver to monitor for short to ground
      driver.s2g_level(6);                  //lower values set drive to be very sensitive to high side voltage swings
    }

    /* minimum settings to to get a motor moving using SPI commands */{
      driver.tbl(2);                          //set blanking time to 24
      driver.toff(driv_toff);                 //pwm off time factor
      driver.pwm_freq(1);                     //pwm at 35.1kHz
    }
  }

  /**********************************************************
   * The last steps before actually moving the motor is setting what ramp mode to use
   * and motion values. ( accel / deccel / velocities)
   * 
   * @ 12MHz clock, a 1.8 motor using 256 micro steps, driver output top speed is:
     7500 RPM / 125 RPS / 45,000 deg/sec / 1,277 mm/sec / 8,388,096 us/sec
     Which takes second from 0 rpm to 7500 RPM at an AMAX setting of 65535 (10.29104296875 mm/s2).
   *************************************************************/
  /* Ramp mode (default)*/{
    driver.RAMPMODE(0);             //set ramp mode to positioning
    Ramp_settings(1,0,0,0,5,25,5,0,0);
  }

  /* Reseting drive faults and re-enabling drive */ {
    digitalWrite(drv_en, HIGH);             //disable drive to clear any start up faults
    delay(1000);                            //give the drive some time to clear faults
    digitalWrite(drv_en, LOW);              //re-enable drive, to start loading in parameters
    driver.GSTAT(7);                        //clear gstat faults
  }

  /**********************************************************************
   *  Start using stallguard to detect skipped steps and stalls
   * 
   *  We will also try and use the stallguard feature to perform motor
   *  characterization curves. So that later we can optimize the motors
   *  acceleration, deceleration, and velocity points in for the ramp generator.
   * 
   * We would normally use tcoolthrs and sgstop, however we cannot reset sg_stop 
   * unless we power down the driver currently. Due to a bug with the library.
   * **************************************************************************/
  
  /* stallguard settings*/
  //driver.TCOOLTHRS(300);
  driver.sg_stop(0);
  driver.sgt(5);
  driver.sfilt(0);
  if (driver.position_reached() == 1) driver.XTARGET((220 / motor_mm_per_microstep));     //verify motor is at starting position, then move motor equivalent to 100mm
  while(homing_count < 4){
    while(driver.position_reached() == 0){
      if(/*driver.stallguard() == 1 ||*/ (driver.VACTUAL() > 100000 && driver.sg_result() == 0)){
        homing_calibrate[homing_count] = driver.XACTUAL();
        driver.XTARGET(homing_calibrate[homing_count]);
        driver.sg_stop(0);
        delay(50);
        driver.sg_stop(1);
        driver.XTARGET(homing_calibrate[homing_count] - 50);
        while(driver.position_reached() == 0);
        if (driver.position_reached() == 1) driver.XTARGET((220 / motor_mm_per_microstep));     //verify motor is at starting position, then move motor equivalent to 100mm
      }
    }

    //if (driver.position_reached() == 1) driver.XTARGET((220 / motor_mm_per_microstep));     //verify motor is at starting position, then move motor equivalent to 100mm
  }

}

void loop() {
  
  //Serial.println("mechanical load , position , time , velocity , accel, jerk");
  Serial.println("Time, Position, Velocity, dT, dx,  Accel, Jerk, Tsteps, Apparat load, Current Coil A, Current Coil B");
  /*Now lets start the first actual move to see if everything worked, and to hear what the stepper sounds like.*/
    if (driver.position_reached() == 1) driver.XTARGET((200 / motor_mm_per_microstep));     //verify motor is at starting position, then move motor equivalent to 100mm
    time = .001;
    dT = 0;
    dx = 0;
    accel = 0;
    jerk = 0;

    while (driver.position_reached() == 0){                                                 //while in motion do nothing. This prevents the code from missing actions
      read_motor_performance();
    }

    if (driver.position_reached() == 1) driver.XTARGET(0);                                  //verify motor is at position, then move motor back to starting position
    while (driver.position_reached() == 0){                                                 //while in motion do nothing. This prevents the code from missing actions
      read_motor_performance();
    }
    while (1);                    //debug message hold to know when program has exited setup routine.
    
} //end of loop
//end of loop

/************************************************************
   Actual base calc values function;
   This function sends the calculated initial values via uart
   to either the arduino monitor, a terminal program, or anything you want
   that uses uart.
   The use of the arduino F() function is critical! to memory usage, without using the F() function,
   all the serial data constant strings would eat up ALL of an arduino megas sram memory.
   With the F() function the over code uses less than 10% of the megas SRAM.
***************************************************************/
void base_calc_values(void) {
  Serial.print(F("Supply voltage set to -> "));
  Serial.println(supply_voltage);
  Serial.println("");
  Serial.print(F("Motor rated milliamps -> "));
  Serial.println(motor_milliamps);
  Serial.print(F("Motor rated volts -> "));
  Serial.println(motor_voltage);
  Serial.print(F("Motor coil resistance -> "));
  Serial.println(motor_resistance);
  Serial.print(F("Motor rated holding torque in mNm -> "));
  Serial.println(motor_hold_torque);
  Serial.println("");
  Serial.print(F("Driver clock frequency -> "));
  Serial.println(drv_clock);
  Serial.println("");
  Serial.print(F("Calculated nominal amps based on motor wattage -> "));
  Serial.println(nominal_amps);
  Serial.print(F("Calculated back EMF constant -> "));
  Serial.println(cbemf);
  Serial.println("");
  Serial.print(F("Microsteps per revolution -> "));
  Serial.println(microsteps_per_rev);
  Serial.println("");
  Serial.print(F("Calculated PMW off time initializer -> "));
  Serial.println(driv_toff);
  Serial.println("");
  Serial.print(F("calculated velocity based PWM gradient -> "));
  Serial.println(drv_pwm_grad);
  Serial.print(F("Calculated initial PWM offset -> "));
  Serial.println(drv_pwm_ofs);  
  Serial.print(F("Drive PWM_SCALE_SUM calculation -> "));
  Serial.println( (drv_pwm_ofs + drv_pwm_grad * .7488) );   //The .7488 = 256 * (step frequency / clock freq)
} //end of base calc
//end of base calc

void read_motor_performance(void){
  /*display measured load values ("Time, Position, Velocity, dT, dx,  Accel, Jerk, Apparat load, Current Coil A, Current Coil B")*/{
    position = (driver.XACTUAL() * motor_mm_per_microstep);
    velocity = (driver.VACTUAL() * motor_mm_per_microstep);
    dT = velocity / position;
    dx = velocity * time;
    accel = velocity  / time;
    jerk = accel  / time;
    tsteps = driver.TSTEP();
    load = constrain(driver.sg_result(),0,1000);
    old_velocity = velocity;
    old_accel = accel;
    //coil_a = ;
    //coil_b = ;

    Serial.print(time,4);
    Serial.print(" , ");
    Serial.print(position,4);
    Serial.print(" , ");
    Serial.print(velocity,4);
    Serial.print(" , ");
    Serial.print(dT,4);
    Serial.print(" , ");
    Serial.print(dx,4);
    Serial.print(" , ");
    Serial.print(accel,4);
    Serial.print(" , ");
    Serial.print(jerk,4);
    Serial.print(" , ");
    if( tsteps < 300) Serial.print(tsteps);
    Serial.print(" , ");
    Serial.print(load,4);
    Serial.print(" , ");
    Serial.print(coil_a,4);
    Serial.print(" , ");
    Serial.println(coil_b,4);
    //Serial.print(" , ");
    time = time + .008;
  }
} //end of read performance
//end of read performance



void Ramp_settings(int units, double vstart, double a1, double v1, double amax, double vmax, double dmax, double d1, double vstop){
  double scale_value;

  uint32_t scaled_vstart,
           scaled_a1,
           scaled_v1,
           scaled_amax,
           scaled_vmax,
           scaled_dmax,
           scaled_d1,
           scaled_vstop; 

  switch(units){
    case 1: {scale_value = motor_mm_per_microstep; break;}           //mm per second
    case 2: {scale_value = motor_mm_per_microstep * 60; break;}      //mm per minute
    case 3: {scale_value = motor_mm_per_microstep / 1000; break;}    // meters per second
    //case 4: {scale_value = }//RPS
    //case 5: {scale_value = }//RPM
    default: {scale_value = 1; break; }//microstep counts
  }      

  scaled_vstart = constrain((vstart / scale_value), 0, 262143);
  scaled_a1 = constrain((a1 / scale_value), 0, 65535);
  scaled_v1 = constrain((v1 / scale_value), 0, 1048575);
  scaled_amax = constrain((amax / scale_value), 0, 65535);
  scaled_vmax = constrain((vmax / scale_value), 0, 8388096);
  scaled_dmax = constrain((dmax / scale_value), 0, 65535);
  scaled_d1 = constrain((d1 / scale_value), 0, 65535);
  scaled_vstop = constrain((vstop / scale_value), 0, 262143);

  if(scaled_vstart > scaled_vstop) scaled_vstop = scaled_vstart;
  if(scaled_vstart > scaled_vmax) scaled_vmax = scaled_vstart;
  if(scaled_d1 < 1) scaled_d1 = 1;
  if(scaled_vstop < 10) scaled_vstop = 10;

  driver.VSTOP(scaled_vstop);              //set stop velocity to 10 steps/sec
  driver.VSTART(scaled_vstart);             //set start velocity to 10 steps/sec
  
  driver.V1(scaled_v1);               //midpoint velocity to  steps/sec ( steps/sec)
  driver.VMAX(scaled_vmax);             //max velocity to  steps/sec ( steps/sec)

  driver.A1(scaled_a1);               //initial accel at  steps/sec2 ( steps/sec2)
  driver.AMAX(scaled_amax);             //max accel at  steps/sec2 ( steps/sec2)

  driver.DMAX(scaled_dmax);             //max deccel  steps/sec2 ( steps/sec2)
  driver.D1(scaled_d1);               //mid deccel  steps/sec2 ( steps/sec2)
}       





//end of program