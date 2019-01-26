#include <Arduino.h>
#include <i2c_t3.h>
#include <Servo.h>
#include <Adafruit_BNO055.h> // Library for reading the 
#include <SBUS.h> // Library for reading SBUS protocol from RC recievers

// TODO fill these in later
// Indexes into the channel array for each command value
#define CH_THROTTLE 0
#define CH_YAW 0
#define CH_PITCH 0
#define CH_ROLL 0
#define CH_LANDING_LOCK 0

// Indexes into the cmd value array 
#define CMD_THROTTLE 0
#define CMD_YAW 1
#define CMD_PITCH 2
#define CMD_ROLL 3

// Indicies into the motor_ppm_values array
#define PF_MOTOR 0
#define SF_MOTOR 1
#define PA_MOTOR 2
#define SA_MOTOR 3

// Indicies into the reference array
#define REF_YAW 0
#define REF_PITCH 1
#define REF_ROLL 2

// Indicies into error arrays
#define ERR_YAW 0
#define ERR_PITCH 1
#define ERR_ROLL 2

#define YAW_RATE_MAX_MAG float(3.14/8) // Max yaw rate in rad/s
#define PITCH_ANGLE_MAX_MAG float(45.0) // Max pitch angle in deg
#define ROLL_ANGLE_MAX_MAG float(45.0) // Max roll angle in deg

// NOTE not currently used. Implement flying with rates later
#define PITCH_RATE_MAX_MAG float(3.14/8) // Max pitch rate in rad/s
#define ROLL_RATE_MAX_MAG float(3.14/8) // Max roll rate in rad/s

#define MAX_PPM_ADJUSTMENT 250 // constrain the addustment to the ppm that can be made by any one orientation adjustment

// Indicies into the gain array
#define IDX_P_GAIN 0
#define IDX_I_GAIN 1
#define IDX_D_GAIN 2
#define IDX_YAW_GAIN 0
#define IDX_PITCH_GAIN 1
#define IDX_ROLL_GAIN 2

Servo PFMotor; // Port fore motor
Servo SFMotor; // Starboard fore motor
Servo PAMotor; // Port aft motor
Servo SAMotor; // Starboard aft motor

SBUS x8r(Serial1); // Object for reading SBUS

Adafruit_BNO055 bno = Adafruit_BNO055(55); // Adafruit object for reading BNO055 breakout

// SBUS data containers
uint16_t channels[16];
bool failSafe;
bool lostFrame;

// Motor PPM values
uint16_t motor_ppm_values[] = {1000, 1000, 1000, 1000};

// Command values recieved from the controler mapped to range [-1.0, 1.0] or [0.0, 1.0] for throttle
float commands[] = {0.0, 0.0, 0.0, 0.0};

// Reference values for the controler
float reference[] = {0.0, 0.0, 0.0};

// PID controller gains
float pid_gains[3][3] = {{1.0, 1.0, 1.0}, // P gains
                         {1.0, 1.0, 1.0}, // I gains
                         {1.0, 1.0, 1.0}};// D gains

// PID error
float error[3] = {0, 0, 0};

// PID error sum
float sum_error[3] = {0, 0, 0};

// PID prev error
float prev_error[3] = {0, 0, 0};


// Is the switch on the controller for landing lock switched to the on position
bool isLandingLockReleased = false;

sensors_event_t bno_event; // Adafruit sensor event that will hold our orientation values

// Display details about the current BNO055 sensor
void displayBNO055Details(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// Display the status of the BNO055 sensor
void displayBNO055Status(void)
{
  // Get the system status values (mostly for debugging purposes) 
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Display the results in the Serial Monitor 
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

// Display the calibration status of the BNO055 sensor
void displayBNO055CalStatus(void)
{
  // Get the four calibration values (0..3) 
  // Any sensor data reporting 0 should be ignored, 
  // 3 means 'fully calibrated" 
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // The data should be ignored until the system calibration is > 0 

  // Display the individual values 
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void setup() {

    // Debugging serial connection
    Serial.begin(9600);

    // Begin the SBUS
    x8r.begin();

    // Begin the BNO055 accel
    if(!bno.begin())
    {
        // There was a problem detecting the BNO055 ... check your connections 
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    
    // Show info about the BNO055 IMU
    displayBNO055Details();
    displayBNO055Status();
    delay(5000);
    displayBNO055CalStatus();

    bno.setExtCrystalUse(true); // Set the BNO055 to use external crystal oscillator

    // Attach servo motor controlers to each of the ESCs
    PFMotor.attach(20);
    SFMotor.attach(21);
    PAMotor.attach(22);
    SAMotor.attach(23);

    // Set the pulse width of the servo motor controlers to 0us so that motors do not arm
    PFMotor.writeMicroseconds(0);
    SFMotor.writeMicroseconds(0);
    PAMotor.writeMicroseconds(0);
    SAMotor.writeMicroseconds(0);

    while(!isLandingLockReleased){
        if(x8r.read(channels, &failSafe, &lostFrame)){
            isLandingLockReleased = true ? channels[CH_LANDING_LOCK] > 1500 : false; // Update landing lock value
        }
        Serial.println("Landing lock active...");

    }

    Serial.println("Landing lock released.");

    /*
    bno.getEvent(&bno_event);
    reference[REF_YAW] = bno_event.orientation.heading;
    */

}


void loop() {

    // Read the values from the SBUS and update command values
    if(x8r.read(channels, &failSafe, &lostFrame)){
        Serial.println("------ SBUS FRAME ------");
        for(int i = 0; i < 16; i++){
            Serial.print("Channel ");
            Serial.print(i);
            Serial.print(" value: ");
            Serial.println(channels[i]);
        }
        Serial.println("------------------------\n");


        // Get commanded values from the radio and store them in the commands array
        commands[CMD_THROTTLE] = map(channels[CH_THROTTLE],1000, 2000, 0.0, 1.0); // TODO may not even need this as pretty much passing it through
        commands[CMD_YAW] = map(channels[CH_YAW], 1000, 2000, -1.0, 1.0);
        commands[CMD_ROLL] = map(channels[CH_ROLL], 1000, 2000, -1.0, 1.0);
        commands[CMD_PITCH] = map(channels[CH_PITCH], 1000, 2000, -1.0, 1.0);

        // Print the values to serial
        Serial.println("------ Current Commands ------");
        Serial.print("Throttle: ");
        Serial.println(commands[CMD_THROTTLE]);
        Serial.print("Yaw: ");
        Serial.println(commands[CMD_YAW]);
        Serial.print("Pitch: ");
        Serial.println(commands[CMD_PITCH]);
        Serial.print("Roll: ");
        Serial.println(commands[CMD_ROLL]);
        Serial.println("------------------------------\n");

    }

    // Update inertial values from the BNO055 IMU
    bno.getEvent(&bno_event);

    // If our throttle reference is less than 25%, then do not do PID
    if(commands[CMD_THROTTLE] < 1250){
        // Write only the throttle value to each motor
        PFMotor.writeMicroseconds(commands[CMD_THROTTLE]);
        SFMotor.writeMicroseconds(commands[CMD_THROTTLE]);
        PAMotor.writeMicroseconds(commands[CMD_THROTTLE]);
        SAMotor.writeMicroseconds(commands[CMD_THROTTLE]);
    } 
    // Otherwise update the PID 
    else{
        // Get reference angle values from commands for roll and pitch
        reference[REF_ROLL] = map(commands[CMD_ROLL], -1.0, 1.0, -ROLL_ANGLE_MAX_MAG, ROLL_ANGLE_MAX_MAG);
        reference[REF_PITCH] = map(commands[CMD_PITCH], -1.0, 1.0, -PITCH_ANGLE_MAX_MAG, PITCH_ANGLE_MAX_MAG);
        
        // Get reference angular rate from commands for yaw
        reference[REF_YAW] = map(commands[CMD_YAW], -1.0, 1.0, -YAW_RATE_MAX_MAG, YAW_RATE_MAX_MAG);

        // Calculate y p r errors
        error[ERR_YAW] = reference[REF_YAW] - bno_event.gyro.heading;
        error[ERR_PITCH] = reference[REF_PITCH] - bno_event.orientation.pitch;
        error[ERR_ROLL] = reference[REF_ROLL] - bno_event.orientation.roll;

        // Calculate error sums
        sum_error[ERR_YAW] += error[ERR_YAW];
        sum_error[ERR_PITCH] += error[ERR_PITCH];
        sum_error[ERR_ROLL] += error[ERR_ROLL];

        float diff_error[3];
        
        // Calculate error diffs
        diff_error[ERR_YAW] = error[ERR_YAW] - prev_error[ERR_YAW];
        diff_error[ERR_PITCH] = error[ERR_PITCH] - prev_error[ERR_PITCH];
        diff_error[ERR_ROLL] = error[ERR_ROLL] - prev_error[ERR_ROLL];

        // Set prev_error to current error
        memcpy(prev_error, error, 3);

        // Adjustments which determine how fast we yaw and how much we pitch or roll
        int16_t yaw_adj, pitch_adj, roll_adj;

        yaw_adj = pid_gains[IDX_P_GAIN][IDX_YAW_GAIN] * error[ERR_YAW] 
            + pid_gains[IDX_I_GAIN][IDX_YAW_GAIN] * sum_error[ERR_YAW] 
            + pid_gains[IDX_D_GAIN][IDX_YAW_GAIN] * diff_error[ERR_YAW];

        pitch_adj = pid_gains[IDX_P_GAIN][IDX_PITCH_GAIN] * error[ERR_PITCH] 
            + pid_gains[IDX_I_GAIN][IDX_PITCH_GAIN] * sum_error[ERR_PITCH] 
            + pid_gains[IDX_D_GAIN][IDX_PITCH_GAIN] * diff_error[ERR_PITCH];

        roll_adj = pid_gains[IDX_P_GAIN][IDX_ROLL_GAIN] * error[ERR_ROLL] 
            + pid_gains[IDX_I_GAIN][IDX_ROLL_GAIN] * sum_error[ERR_ROLL] 
            + pid_gains[IDX_D_GAIN][IDX_ROLL_GAIN] * diff_error[ERR_ROLL];

        uint16_t new_throttle = map(commands[CMD_THROTTLE], 0.0, 1.0, 1000, 2000);

        // Calculate and constrain new motor commands
        motor_ppm_values[PF_MOTOR] = constrain(new_throttle + yaw_adj + pitch_adj + roll_adj, 1000, 2000);
        motor_ppm_values[SF_MOTOR] = constrain(new_throttle - yaw_adj + pitch_adj - roll_adj, 1000, 2000);
        motor_ppm_values[PA_MOTOR] = constrain(new_throttle - yaw_adj - pitch_adj + roll_adj, 1000, 2000);
        motor_ppm_values[SA_MOTOR] = constrain(new_throttle + yaw_adj - pitch_adj - roll_adj, 1000, 2000);
    }

    /*

    uint16_t new_throttle = map(commands[CMD_THROTTLE], 0.0, 1.0, 1000, 2000);

    uint16_t yaw_adj = pid_gains[IDX_P_GAIN][IDX_YAW_GAIN] * (reference[REF_YAW] - bno_event.gyro.z);

    uint16_t roll_adj = pid_gains[IDX_P_GAIN][IDX_ROLL_GAIN] * (reference[REF_ROLL] - bno_event.orientation.roll);
    uint16_t pitch_adj = pid_gains[IDX_P_GAIN][IDX_PITCH_GAIN] * (reference[REF_PITCH] - bno_event.orientation.pitch);
    
    yaw_offset += yaw_adj; // Add the signed adjustment to the yaw offset
    yaw_offset = constrain(yaw_offset, -MAX_PPM_ADJUSTMENT, MAX_PPM_ADJUSTMENT);


    motor_ppm_values[PF_MOTOR] = constrain(new_throttle + yaw_adj + pitch_adj + roll_adj, 1000, 2000);
    motor_ppm_values[SF_MOTOR] = constrain(new_throttle - yaw_adj + pitch_adj - roll_adj, 1000, 2000);
    motor_ppm_values[PA_MOTOR] = constrain(new_throttle - yaw_adj - pitch_adj + roll_adj, 1000, 2000);
    motor_ppm_values[SA_MOTOR] = constrain(new_throttle + yaw_adj - pitch_adj - roll_adj, 1000, 2000);

    // Write the new values to the to each motor
    PFMotor.writeMicroseconds(motor_ppm_values[PF_MOTOR]);
    SFMotor.writeMicroseconds(motor_ppm_values[SF_MOTOR]);
    PAMotor.writeMicroseconds(motor_ppm_values[PA_MOTOR]);
    SAMotor.writeMicroseconds(motor_ppm_values[SA_MOTOR]);

    */
}