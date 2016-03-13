// SINGLE PID FLIGHT CONTROLLER

//Libraries for the reading the IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>        // Used to control ESC's
#include <PinChangeInt.h> // Include the PinChangeInt library so that I can access more then 2 pints with interrupts

/* This driver reads raw data from the BNO055
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
*/

// Min and max values for ESC's
#define ESC_START 10.0
#define ESC_MIN 62.0
#define ESC_MAX 113.0

//Define The Pins
#define AUX1_PIN 0 // get an error when assigning stuff to pin 0
#define GEAR_PIN 1
#define RUDD_PIN 2
#define ELEV_PIN 3
#define AILE_PIN 4
#define THRO_PIN 7
#define FR_PIN 5
#define FL_PIN 6
#define BR_PIN 8
#define BL_PIN 9

// Define min and max values for the radio
#define RUDD_MAX 1090
#define RUDD_MIN 1908
#define ELEV_MAX 1090
#define ELEV_MIN 1908
#define AILE_MAX 1090
#define AILE_MIN 1908
#define THRO_MAX 1090
#define THRO_MIN 1908

// Bit flags set to indicate which channels have new signals
#define AUX1_FLAG 1
#define GEAR_FLAG 2
#define RUDD_FLAG 4
#define ELEV_FLAG 8
#define AILE_FLAG 16
#define THRO_FLAG 32
// The shared flags
volatile uint16_t Shared_Flags;

// These will hole the actual time values that I get 
volatile uint16_t AUX1_VALUE;
volatile uint16_t GEAR_VALUE;
volatile uint16_t RUDD_VALUE;
volatile uint16_t ELEV_VALUE;
volatile uint16_t AILE_VALUE;
volatile uint16_t THRO_VALUE;

// These are used by the ISR to find the start time

uint32_t StartTime_AUX1;
uint32_t StartTime_GEAR;
uint32_t StartTime_RUDD;
uint32_t StartTime_ELEV;
uint32_t StartTime_AILE;
uint32_t StartTime_THRO;

double RUDD_mapped;
double ELEV_mapped;
double AILE_mapped;
double THRO_mapped;

char temp_text[128];

Adafruit_BNO055 bno = Adafruit_BNO055();

// Setting up typedef for PID's
typedef struct {
  double Output;
  unsigned long Last_time;
  double Sample_time;
  double Setpoint;
  double Input;
  double Error_sum;
  double Last_error;
  double kp;
  double ki;
  double kd;
  } PID_type;

//initialize position pids
PID_type Roll_position;
PID_type Pitch_position;
PID_type Yaw_position;

//Setup the servo motors to contorl the ESC's
Servo FR; // Front Right
Servo FL; // Front Left
Servo BR; // Back Right
Servo BL; // Back Left

void setup(void)
{
  Serial.begin(9600);
  
  ////////// Initializing Interrupts \\\\\\\\\\
  // Attaches the interrupts to the selected pin using the PinChangeInt library
  PCintPort::attachInterrupt(AUX1_PIN, calculate_AUX1,CHANGE);
  PCintPort::attachInterrupt(GEAR_PIN, calculate_GEAR,CHANGE);
  PCintPort::attachInterrupt(RUDD_PIN, calculate_RUDD,CHANGE);
  PCintPort::attachInterrupt(ELEV_PIN, calculate_ELEV,CHANGE);
  PCintPort::attachInterrupt(AILE_PIN, calculate_AILE,CHANGE);
  PCintPort::attachInterrupt(THRO_PIN, calculate_THRO,CHANGE);

  ////////// Initializing PID's \\\\\\\\\\
  double kp = .2;
  double Input = 0;
  double ki = .1;
  double kd = 0;
  double Sample_time = 200;
  double Setpoint = 0;
  Serial.println("Initializing Position PIDs...");
  
  Serial.print("Roll_position...");
  Roll_position = initialize_pid(Roll_position, kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println("Completed");
  Serial.print("Pitch_position...");
  Pitch_position = initialize_pid(Pitch_position, kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println("Completed");
  Serial.print("Yaw_position ...");
  Yaw_position = initialize_pid(Yaw_position, kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println(" Completed");

  ////////// Setup IMU \\\\\\\\\\
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  //  Initialise the sensor
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  ////////// Setup Motors and ESC's \\\\\\\\\\
  Serial.print("Setting up motor assignments...");
  // Setup Motor Pin Assignments
  FR.attach(FR_PIN);
  FL.attach(FL_PIN);
  BR.attach(BR_PIN);
  BL.attach(BL_PIN);
  Serial.println("Motor setup complete");

  // Printing startup message
  Serial.println("initializing ESC's");
  // Write initial values to ESC's so they don't do weird shit
  FR.write(ESC_START);
  FL.write(ESC_START);
  BR.write(ESC_START);
  BL.write(ESC_START);
  delay(3000); // Wait for ESC's to warm up
  Serial.println("ESC Ready");
}

void loop(void)
{
  ////////// Interrupt Stuff \\\\\\\\\\
  static uint16_t AUX1_Uninterupted_Value;
  static uint16_t GEAR_Uninterupted_Value;
  static uint16_t RUDD_Uninterupted_Value;
  static uint16_t ELEV_Uninterupted_Value;
  static uint16_t AILE_Uninterupted_Value;
  static uint16_t THRO_Uninterupted_Value;
  static uint16_t Local_Flags;

  if (Shared_Flags)
  {
 
    // Turn off the interupts while we do important shit
    // Like writing data to write to motors
    noInterrupts();

    Local_Flags = Shared_Flags;

    if (Local_Flags & AUX1_FLAG)
    {
      AUX1_Uninterupted_Value = AUX1_VALUE;
    }

    if (Local_Flags & GEAR_FLAG)
    {
      GEAR_Uninterupted_Value = GEAR_VALUE;
    }

    if (Local_Flags & RUDD_FLAG)
    {
      RUDD_Uninterupted_Value = RUDD_VALUE;
    }

    if (Local_Flags & ELEV_FLAG)
    {
      ELEV_Uninterupted_Value = ELEV_VALUE;
    }

    if (Local_Flags & AILE_FLAG)
    {
      AILE_Uninterupted_Value = AILE_VALUE;
    }

    if (Local_Flags & THRO_FLAG)
    {
      THRO_Uninterupted_Value = THRO_VALUE;
    }
  }
  
  interrupts();

  RUDD_mapped = map(RUDD_Uninterupted_Value, RUDD_MIN ,RUDD_MAX, -180, 180); // Yaw
  ELEV_mapped = map(ELEV_Uninterupted_Value, ELEV_MIN ,ELEV_MAX, 45, -45); // Pitch
  AILE_mapped = map(AILE_Uninterupted_Value, AILE_MIN ,AILE_MAX, 45, -45); // Roll
  THRO_mapped = map(THRO_Uninterupted_Value, THRO_MIN ,THRO_MAX,  100, 0);  // Elevation

  ////////// IMU STUFF \\\\\\\\\\
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Yaw_position.Input = euler.x();     //Pitch is z axis
  Roll_position.Input = euler.y();    //Roll is y axis
  Pitch_position.Input = euler.z();   // Yaw is x axis


  ////////// Calculate PID's \\\\\\\\\\  
  // setpoint for stability PID's
  Roll_position.Setpoint =  AILE_mapped;
  Yaw_position.Setpoint = RUDD_mapped;
  Pitch_position.Setpoint = ELEV_mapped;

  Roll_position = Compute(Roll_position);
  Yaw_position = Compute(Yaw_position);
  Pitch_position = Compute(Pitch_position);

  ////////// Output Motor Calculations \\\\\\\\\\
  
  double BL_value = (THRO_mapped - Roll_position.Output + Pitch_position.Output) + Yaw_position.Output;
  double FR_value = (THRO_mapped + Roll_position.Output - Pitch_position.Output) + Yaw_position.Output;
  double BR_value = (THRO_mapped + Roll_position.Output + Pitch_position.Output) - Yaw_position.Output;
  double FL_value = (THRO_mapped - Roll_position.Output - Pitch_position.Output) - Yaw_position.Output; 
  FL_value = map(FL_value, -45, 45, ESC_MIN, ESC_MAX);
  FL_value = constrain(FL_value, ESC_MIN, ESC_MAX); 
  
  FR_value = map(FL_value, -45, 45, ESC_MIN, ESC_MAX);
  FR_value = constrain(FL_value, ESC_MIN, ESC_MAX); 
    
  FL.write(FL_value);
  FR.write(FR_value);
//  Serial.print("Motor_Value");
//  Serial.println(FL_value);
  
//  BL.write(constrain(BL_value, ESC_MIN, ESC_MAX));
//  FR.write(constrain(FR_value, ESC_MIN, ESC_MAX));
//  BR.write(constrain(BR_value, ESC_MIN, ESC_MAX));

  print_pid(Roll_position);
 // double FR_val = constrain(map(Roll_rate.Output, 0, 30, ESC_MIN, ESC_MAX),ESC_MIN, ESC_MAX);
  
//  if (Shared_Flags > 0)
//  {
//    sprintf(temp_text, "RUDD_VALUE: %4d ELEV_VALUE: %4d AILE_VALUE: %4d THRO_VALUE: %4d", RUDD_mapped, ELEV_mapped, AILE_mapped, THRO_mapped);
//      Serial.println(temp_text);
//  }

// Print Commands
// print_IMU(euler, GYROSCOPE);
// print_pid(Roll_position);

}


void print_IMU(imu::Vector<3> euler, imu::Vector<3> GYROSCOPE)
{
// Print the Absolute Orientation Angle
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
// Print the Gyroscopic acceleration data
  Serial.print("xdot: ");
  Serial.print(GYROSCOPE.x());
  Serial.print(" ydot: ");
  Serial.print(GYROSCOPE.y());
  Serial.print(" zdot: ");
  Serial.print(GYROSCOPE.z());
  Serial.print("\t\t");
  Serial.println(""); 
}

PID_type initialize_pid(PID_type controller, double kp, double ki, double kd, double Sample_time, double Setpoint, double Input)
{
  Serial.println("Initializing PID...");
  controller.Output = 0;
  controller.Last_time = 0;
  controller.Sample_time = Sample_time;
  controller.Setpoint = 0;
  controller.Input = 0;
  controller.Error_sum = 0;
  controller.Last_error = 0;
  controller.kp = kp;
  controller.ki = ki;
  controller.kd = kd;

  print_pid(controller);
  Serial.println("PID completed.");  
  return controller;
}

PID_type Compute(PID_type controller)
{
  unsigned long now = millis();  // get current time
  int Time_change = now - controller.Last_time;  // calculate time change from last

  if(Time_change >= controller.Sample_time)        //  make sure PID runs only after certain amount of time
  {
//    Serial.println("PID RUNNING..");
    double error = controller.Setpoint - controller.Input;
    controller.Error_sum += error;

    double dError = (error - controller.Last_error);
    double Proportional_term = controller.kp * error;
    double Integral_term = controller.kp * error;
    double Derivative_term= controller.kp * error;
    controller.Output = Proportional_term + Integral_term + Derivative_term;    
    controller.Last_error = error;
    controller.Last_time = millis();
  }
  return controller;
}


void print_pid(PID_type controller)
{

//  Serial.println("The PID values are: ");
  Serial.print("Output: ");
  Serial.print(controller.Output);

  Serial.print("\tLast Time: ");
  Serial.print(controller.Last_time);

  Serial.print("\tSample_time: ");
  Serial.print(controller.Sample_time);

  Serial.print("\tSetpoint: ");
  Serial.print(controller.Setpoint);

  Serial.print("\tInput: ");
  Serial.print(controller.Input);

  Serial.print("\tError Sum: ");
  Serial.print(controller.Error_sum);

  Serial.print("\tLast Error: ");
  Serial.print(controller.Last_error);
  Serial.println("");
}


void calculate_AUX1()
{
  if (digitalRead(AUX1_PIN) == HIGH)
  {
    // If the pin is high then it is rising
    // Start the timer
    StartTime_AUX1 = micros();
  }
  else
  {
    // If it isn't rising it must be falling otherwise the interrupt wouldn't be called
    AUX1_VALUE = (uint16_t)(micros() - StartTime_AUX1);
    // Sets the new flag to show loop that new signal has been received
    Shared_Flags = Shared_Flags | AUX1_FLAG;
  }
}


void calculate_GEAR()
{
  if (digitalRead(GEAR_PIN) == HIGH)
  {
    StartTime_GEAR = micros();
  }
  else
  {
    GEAR_VALUE = (uint16_t)(micros() - StartTime_GEAR);
    Shared_Flags = Shared_Flags | GEAR_FLAG;
  }
}

void calculate_RUDD()
{
  if (digitalRead(RUDD_PIN) == HIGH)
  {
    StartTime_RUDD = micros();
  }
  else
  {
    RUDD_VALUE = (uint16_t)(micros() - StartTime_RUDD);
    Shared_Flags = Shared_Flags | RUDD_FLAG;
  }
}

void calculate_ELEV()
{
  if (digitalRead(ELEV_PIN) == HIGH)
  {
    StartTime_ELEV = micros();
  }
  else
  {
    ELEV_VALUE = (uint16_t)(micros() - StartTime_ELEV);
    Shared_Flags = Shared_Flags | ELEV_FLAG;
  }
}

void calculate_AILE()
{
  if (digitalRead(AILE_PIN) == HIGH)
  {
    StartTime_AILE = micros();
  }
  else
  {
    AILE_VALUE = (uint16_t)(micros() - StartTime_AILE);
    Shared_Flags = Shared_Flags | AILE_FLAG;
  }
}

void calculate_THRO()
{
  if (digitalRead(THRO_PIN) == HIGH)
  {
    StartTime_THRO = micros();
  }
  else
  {
    THRO_VALUE = (uint16_t)(micros() - StartTime_THRO);
    Shared_Flags = Shared_Flags | THRO_FLAG;
  }
}