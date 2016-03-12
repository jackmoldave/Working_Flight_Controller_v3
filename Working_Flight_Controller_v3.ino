//Parts of the flight controller:
//
//Interrupts / radio reading
//PID - theoreticlaly working
//Motor writing
//Reading IMU - theoretically working

//time to put the motor writting into this

//Libraries for the reading the IMU
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>        // Used to control ESC's

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

//initialize rate pids
PID_type Roll_rate;
PID_type Pitch_rate;
PID_type Yaw_rate;

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
  ////////// Initializing PID's \\\\\\\\\\

  double Input = 0;   
  double kp = .2;
  double ki = .1;
  double kd = 0;
  double Sample_time = 200;
  double Setpoint = 0;
  Serial.println("Initializing Rate PIDs...");
  Serial.print(" Roll_rate...");
  Roll_rate = initialize_pid(Roll_rate, kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println(" Completed");
  Serial.print(" Pitch_rate...");
  Pitch_rate = initialize_pid(Pitch_rate, kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println(" Completed");
  Serial.print(" Yaw_rate...");
  Yaw_rate = initialize_pid(Yaw_rate , kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println(" Completed");
  
  
  Serial.println("Initializing Position PIDs...");
  Serial.print("Roll_rate...");
  Roll_position = initialize_pid(Roll_position, kp, ki, kd, Sample_time, Setpoint, Input);
  Serial.println("Completed");
  
  Serial.print("Pitch_rate...");
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

  Serial.print("Setting up motor assignments...");
  // Setup Motor Pin Assignments
  FR.attach(9);
  FL.attach(10);
  BR.attach(11);
  BL.attach(12);
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

  Yaw_rate.Input = gyroscope.x();     //Pitch is z axis
  Roll_rate.Input = gyroscope.y();    //Roll is y axis
  Pitch_rate.Input = gyroscope.z();   // Yaw is x axis
  
  
  Roll_position = Compute(Roll_position);
  Yaw_position = Compute(Yaw_position);
  Pitch_position = Compute(Pitch_position);

  
  Roll_rate.Setpoint = Roll_position.Output;
  Pitch_rate.Setpoint= Pitch_position.Output;
  Yaw_rate.Setpoint = Yaw_position.Output;

  Roll_rate = Compute(Roll_rate);
  Yaw_rate = Compute(Yaw_rate);
  Pitch_rate = Compute(Pitch_rate);
  
  




//  motor FL: throttle - roll_output - pitch_output - yaw_output
//  motor BL: throttle - roll_output + pitch_output _ yaw_output
//  motor FR: rcthr + roll_output - pitch output + yaw_output
//  motor BR: rcthr + roll_output + pitch_output - yaw_output

  print_pid(Roll_rate);
  double FR_val = constrain(map(Roll_rate.Output, 0, 30, ESC_MIN, ESC_MAX),ESC_MIN, ESC_MAX);
  Serial.println(FR_val);
  FR.write(FR_val);

////////  MOTOR STUFF \\\\\\\\\\
// Writ eto the serveo
//FR.write(val);
//FL.write(val);
//BR.write(val);
//BL.write(val);
  
  // print the absolute angle and the angle acceleration
  // print_IMU(euler, GYROSCOPE);
  // Print the PID values
//  print_pid(Roll_position);
//  Serial.println(Roll_position.Last_time);

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
