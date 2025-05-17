#include "BLECStringCharacteristic.h"
#include "EString.h"
#include <math.h>
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "SparkFun_VL53L1X.h"

#include <BasicLinearAlgebra.h>
using namespace BLA;

#define SERIAL_PORT Serial
#define AD0_VAL 1 // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "2bb357d9-bff9-49ae-92fe-7935d8da4d69"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define ICM_20948_USE_DMP

#define motorR1 7  // 16 //15 //7//A0 //15
#define motorR2 9  // 15 //16 //9//A1 //16
#define motorL2 16 // 2 //1 //16 //A3 //1
#define motorL1 15 // 2 //15 //A5 //2   forward

//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// ToF sensor data arrays
const int num_data_msgs = 500;
float acc_pitch_raw[num_data_msgs];
float acc_pitch_lpf[num_data_msgs];
float acc_roll_raw[num_data_msgs];
float acc_roll_lpf[num_data_msgs];

float gyro_pitch_raw[num_data_msgs];
float gyro_roll_raw[num_data_msgs];
float gyro_yaw_raw[num_data_msgs];

float comp_pitch[num_data_msgs];
float comp_roll[num_data_msgs];
float comp_yaw[num_data_msgs];

float distance1_data[num_data_msgs];
float distance2_data[num_data_msgs];
float pwm_data[num_data_msgs];
float kf_distance_data[num_data_msgs];

float target_angles[num_data_msgs];

// time array
int times[num_data_msgs];

int num_vars_data_collection;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
float temps[num_data_msgs];

// pwm/driving tuning vars
float calibration_factor = 1.0; // 0.8\g
// int pwm = 255;

////////// PID /////////////
bool pid_on = false;
int pid_i; // index
float start_time;
float end_time;
float current_time;
int error_sum = 0;
float target = 304.8;
int max_speed = 200;
float previous_error = 0;

float Kp = 0.04; // 0.08;
float Ki = 0.0004;
float Kd = 90.0;

////////// PID Ori /////////////

float Kp_ori = 0.8;
float Ki_ori = 0.01;
float Kd_ori = 0.7;

bool pid_ori_on = false;
int pid_ori_i;

double error_ori;
double prev_error_ori;
double error_sum_ori;
float yaw;
int pid_speed;
float prev_time;
double current_angle;

float current_pwm;
float target_angle = 0;

int pid_times[num_data_msgs];
double pid_current_angles[num_data_msgs];
double pid_target_angles[num_data_msgs];
int pid_speeds[num_data_msgs];
double pid_err[num_data_msgs];
double pid_ps[num_data_msgs];
double pid_is[num_data_msgs];
double pid_ds[num_data_msgs];

int b = 0;
int i = 0;

int data_i;
bool move = false;

////////// Kalman Filter /////////////
float d = 0.4347826086956522;  // drag
float m = 0.22658842534082704; // momentum
float dt = 0.06;
float kf_speed;
float kf_dist;

// A, B, C matrices
Matrix<2, 2> A = {0, 1,
                  0, -d / m};

Matrix<2, 1> B = {0, 1 / m};

Matrix<1, 2> C = {1, 0};

Matrix<2, 2> Id = {1, 0,
                   0, 1};

Matrix<2, 2> Ad = {1, 0.06,
                   0, 0.88487075};

Matrix<2, 1> Bd = {0, 0.26479729};

// Process and measurement noise
Matrix<2, 2> sigma_u = {1666.66666667, 0,
                        0, 1666.66666667};

Matrix<1, 1> sigma_z = {100};

// initial state covariance
Matrix<2, 2> sigma = {400, 0,
                      0, 100};

// initial state mean
Matrix<2, 1> x = {3774, 0};

////////// Mapping /////////////
// bool observe_map = false;
int increment;
int num_readings;
int pid_ori_error_threshold = 5;

#define XSHUT 10
#define ADDRESS 0x30

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);

//////////// Lab 12 Stuff ////////////

float target_x;
float target_y;

float start_x;
float start_y;

bool first_time;
bool do_navigation;

int max_duration;
int max_straight;
int max_angle;

//////////// Global Variables ////////////

enum CommandTypes
{
  START_CAR,
  START_PID,
  STOP_PID,
  CHANGE_GAIN,
  GET_PID_DATA,
  GET_PITCH_DATA,
  GET_ROLL_DATA,
  GET_YAW_DATA,
  GET_IMU_DATA,
  GET_TOF_DATA,
  START_PID_ORI,
  STOP_PID_ORI,
  START_MAP,
  GET_PID_DATA_ORI,
  FIND_STEADY_STATE,
  GET_KF_DATA,
  STUNT,
  NAVIGATION,
};

void set_up_tof()
{
  Serial.println("VL53L1X Qwiic Test");

  // set up tof sensors
  digitalWrite(XSHUT, LOW);
  distanceSensor1.setI2CAddress(ADDRESS);
  Serial.print("Distance Sensor 1 Address: 0x");
  Serial.println(distanceSensor1.getI2CAddress(), HEX);

  if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  digitalWrite(XSHUT, HIGH);
  Serial.print("Distance Sensor 2 Address: 0x");
  Serial.println(distanceSensor1.getI2CAddress(), HEX);

  if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensors 1 and 2 online!");
}

void handle_command()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
   * since it uses strtok internally (refer RobotCommand.h and
   * https://www.cplusplus.com/reference/cstring/strtok/)
   */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success)
  {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type)
  {
  case START_CAR:
  {
    float PWM_VAL;
    // Extract the next value from the command string as a float
    success = robot_cmd.get_next_value(PWM_VAL);
    if (!success)
      return;

    Serial.println(PWM_VAL);
    unsigned long startTime = millis(); // Record the start time
    drive(1, PWM_VAL);
    break;
  }

  case START_PID:
  {
    set_up_tof();

    pid_i = 0;
    start_time = (float)millis();
    error_sum = 0;
    distanceSensor1.startRanging();

    pid_on = true;
    break;
  }

  case STOP_PID:
  {
    pid_on = false;
    drive(0, 0);
    num_vars_data_collection = pid_i;
    break;
  }

  case CHANGE_GAIN:
  {
    float new_kp;
    float new_ki;
    float new_kd;

    success = robot_cmd.get_next_value(new_kp);
    if (!success)
      return;

    success = robot_cmd.get_next_value(new_ki);
    if (!success)
      return;

    success = robot_cmd.get_next_value(new_kd);
    if (!success)
      return;

    Kp = new_kp;
    Ki = new_ki;
    Kd = new_kd;

    break;
  }

  case GET_PID_DATA:
  {
    for (int i = 0; i < num_vars_data_collection; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(distance1_data[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(pwm_data[i]);
      Serial.println(tx_estring_value.c_str());
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case GET_PITCH_DATA:
  {
    float pitch_g = 0, dt = 0;
    unsigned long last_time = millis();
    // collect pitch data
    for (int i = 0; i < num_data_msgs; i++)
    {
      if (myICM.dataReady())
      {
        myICM.getAGMT();
        // accelerometer data
        float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
        acc_pitch_raw[i] = pitch_a;

        // gyro data
        dt = (millis() - last_time) / 1000.;
        last_time = millis();
        pitch_g = pitch_g + myICM.gyrY() * dt;
        gyro_pitch_raw[i] = pitch_g;

        // time
        times[i] = (int)millis();
      }
    }

    // dt = 1/(sampling rate), RC = 1/(2*pi*cutoff freq), and alpha=dt/(dt+RC)
    const float alpha = 0.0735;

    acc_pitch_lpf[0] = acc_pitch_raw[0];
    comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];

    for (int n = 1; n < num_data_msgs; n++)
    {
      float pitch_raw_curr = acc_pitch_raw[n];
      acc_pitch_lpf[n] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[n - 1];
      acc_pitch_lpf[n - 1] = acc_pitch_lpf[n];

      comp_pitch[n] = (1 - alpha) * gyro_pitch_raw[n] + alpha * acc_pitch_lpf[n];
    }

    // send pitch data from the pitch array to python
    for (int i = 0; i < num_data_msgs; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(acc_pitch_raw[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(acc_pitch_lpf[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(gyro_pitch_raw[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(comp_pitch[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }

    break;
  }

  case GET_ROLL_DATA:
  {
    float roll_g = 0, dt = 0;
    unsigned long last_time = millis();
    // collect roll data
    for (int i = 0; i < num_data_msgs; i++)
    {
      if (myICM.dataReady())
      {
        myICM.getAGMT();
        // acc data
        float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
        acc_roll_raw[i] = roll_a;

        // gyro data
        dt = (millis() - last_time) / 1000.;
        last_time = millis();
        roll_g = roll_g + myICM.gyrX() * dt;
        gyro_roll_raw[i] = roll_g;

        // time
        times[i] = (int)millis();
      }
    }

    // dt = 1/(sampling rate), RC = 1/(2*pi*cutoff freq), and alpha=dt/(dt+RC)
    const float alpha = 0.0735;

    acc_roll_lpf[0] = acc_roll_raw[0];
    comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];

    for (int n = 1; n < num_data_msgs; n++)
    {
      float roll_raw_curr = acc_roll_raw[n];
      acc_roll_lpf[n] = alpha * roll_raw_curr + (1 - alpha) * acc_roll_lpf[n - 1];
      acc_roll_lpf[n - 1] = acc_roll_lpf[n];

      comp_roll[n] = (1 - alpha) * gyro_roll_raw[n] + alpha * acc_roll_lpf[n];
    }

    // send roll data from the roll array to python
    for (int i = 0; i < num_data_msgs; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(acc_roll_raw[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(acc_roll_lpf[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(gyro_roll_raw[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(comp_roll[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }

    break;
  }

  case GET_YAW_DATA:
  {
    float yaw_g = 0, dt = 0;
    unsigned long last_time = millis();
    const float alpha = 0.0735;
    // collect roll data
    for (int i = 0; i < num_data_msgs; i++)
    {
      if (myICM.dataReady())
      {
        myICM.getAGMT();
        // gyro data
        dt = (millis() - last_time) / 1000.;
        last_time = millis();
        yaw_g = yaw_g + myICM.gyrZ() * dt;
        gyro_yaw_raw[i] = yaw_g;
        comp_yaw[i] = (1 - alpha) * gyro_yaw_raw[i];

        // time
        times[i] = (int)millis();
      }
    }

    // send roll data from the roll array to python
    for (int i = 0; i < num_data_msgs; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(gyro_yaw_raw[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(comp_yaw[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }

    break;
  }

  case GET_IMU_DATA:
  {
    float yaw_g = 0, roll_g = 0, pitch_g = 0, dt = 0;
    unsigned long last_time = millis();
    const float alpha = 0.0735;
    // collect roll data
    for (int i = 0; i < num_data_msgs; i++)
    {
      if (myICM.dataReady())
      {
        myICM.getAGMT();
        // acc roll data
        float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
        acc_roll_raw[i] = roll_a;

        // gyro roll data
        dt = (millis() - last_time) / 1000.;
        last_time = millis();
        roll_g = roll_g + myICM.gyrX() * dt;
        gyro_roll_raw[i] = roll_g;

        // accelerometer data
        float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
        acc_pitch_raw[i] = pitch_a;

        // gyro data
        dt = (millis() - last_time) / 1000.;
        last_time = millis();
        pitch_g = pitch_g + myICM.gyrY() * dt;
        gyro_pitch_raw[i] = pitch_g;

        // gyro data
        dt = (millis() - last_time) / 1000.;
        last_time = millis();
        yaw_g = yaw_g + myICM.gyrZ() * dt;
        gyro_yaw_raw[i] = yaw_g;
        comp_yaw[i] = (1 - alpha) * gyro_yaw_raw[i];

        // time
        times[i] = (int)millis();
      }
    }

    // low pass filters

    acc_pitch_lpf[0] = acc_pitch_raw[0];
    comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];
    acc_roll_lpf[0] = acc_roll_raw[0];
    comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];

    for (int n = 1; n < num_data_msgs; n++)
    {
      // lpf pitch
      float pitch_raw_curr = acc_pitch_raw[n];
      acc_pitch_lpf[n] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[n - 1];
      acc_pitch_lpf[n - 1] = acc_pitch_lpf[n];

      comp_pitch[n] = (1 - alpha) * gyro_pitch_raw[n] + alpha * acc_pitch_lpf[n];

      // lpf roll
      float roll_raw_curr = acc_roll_raw[n];
      acc_roll_lpf[n] = alpha * roll_raw_curr + (1 - alpha) * acc_roll_lpf[n - 1];
      acc_roll_lpf[n - 1] = acc_roll_lpf[n];

      comp_roll[n] = (1 - alpha) * gyro_roll_raw[n] + alpha * acc_roll_lpf[n];
    }

    // send roll data from the roll array to python
    for (int i = 0; i < num_data_msgs; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(acc_pitch_raw[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(acc_pitch_lpf[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(acc_roll_raw[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(acc_roll_lpf[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(gyro_pitch_raw[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(gyro_roll_raw[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(gyro_yaw_raw[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(comp_pitch[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(comp_roll[i]);
      tx_estring_value.append(" | ");
      tx_estring_value.append(comp_yaw[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }

    break;
  }

  case GET_TOF_DATA:
  {

    Serial.println("VL53L1X Qwiic Test");

    // set up tof sensors
    // digitalWrite(SHUTDOWN_PIN, LOW);
    distanceSensor1.setI2CAddress(ADDRESS);
    Serial.print("Distance Sensor 1 Address: 0x");
    Serial.println(distanceSensor1.getI2CAddress(), HEX);

    if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
    {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }

    // digitalWrite(SHUTDOWN_PIN, HIGH);
    // Serial.print("Distance Sensor 2 Address: 0x");
    // Serial.println(distanceSensor2.getI2CAddress(), HEX);

    // if (distanceSensor2.begin() != 0) // Begin returns 0 on a good init
    // {
    //     Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    //     while (1)
    //         ;
    // }
    // Serial.println("Sensors 1 and 2 online!");

    distanceSensor1.setDistanceModeLong();
    // distanceSensor2.setDistanceModeShort();

    distanceSensor1.startRanging();
    // distanceSensor2.startRanging();

    int st = millis();

    for (int i = 0; i < num_data_msgs; i++)
    {

      while (!distanceSensor1.checkForDataReady())
      {
        delay(1);
      }
      distance1_data[i] = distanceSensor1.getDistance();
      // Serial.print("Distance 1: ");
      // Serial.print(distance1_data[i]);

      // while (!distanceSensor2.checkForDataReady())
      // {
      //     delay(1);
      // }
      // distance2_data[i] = distanceSensor2.getDistance();
      // Serial.print(" | Distance 2: ");
      // Serial.println(distance2_data[i]);

      times[i] = millis();
    }

    int et = millis();

    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    // distanceSensor2.clearInterrupt();
    // distanceSensor2.stopRanging();

    // Serial.print("Time for 500 sensor readings: ");
    // Serial.println(et-st);

    // st = millis();

    // for(int i = 0; i < num_data_msgs; i++)
    // {
    //   continue;
    // }

    // et = millis();

    // Serial.print("Time for 500 iterations of empty loop: ");
    // Serial.println(et-st);

    for (int i = 0; i < num_data_msgs; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(distance1_data[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(distance2_data[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case START_PID_ORI:
  {
    target_angle = 0;
    pid_ori_i = 0;

    start_time = (float)millis();
    prev_time = (float)millis();

    error_ori = 0;
    prev_error_ori = 0;
    error_sum_ori = 0;

    current_angle = target_angle;
    yaw = 0;

    pid_ori_on = true;

    break;
  }

  case STOP_PID_ORI:
  {
    pid_ori_on = false;
    num_vars_data_collection = pid_ori_i;
    drive(0, 0);
    break;
  }

  case START_MAP:
  {
    // setup
    set_up_tof();
    pid_ori_i = 0;
    increment = 20;
    num_readings = 1;
    start_time = (float)millis();

    // initialize control variables
    pid_ori_error_threshold = 5;
    error_ori = 0;
    prev_error_ori = 0;
    error_sum_ori = 0;
    current_angle = 0;

    distanceSensor1.startRanging();

    while ((float(millis()) - start_time) < 3000)
    {
      pid_ori(180);
    }

    for (int i = -180; i < 180; i += increment)
    {
      target_angle = i;
      // Serial.print("target angle: ");
      // Serial.println(target_angle);
      do
      {
        pid_ori(target_angle);
        // Serial.print("doing pid, error = ");
        // Serial.println(error_ori);
        delay(10);
      } while (!(abs(error_ori) < pid_ori_error_threshold));

      drive(0, 0);
      delay(10);

      int j = 0;
      float sum_dist = 0;

      while (j < num_readings)
      {
        if (distanceSensor1.checkForDataReady() && myICM.dataReady() && pid_ori_i < 1000)
        {
          float yaw_g = 0, roll_g = 0, pitch_g = 0, dt = 0;
          unsigned long last_time = millis();
          if (j != 0)
          {
            last_time = times[pid_ori_i - 1];
          }
          const float alpha = 0.0735;

          times[pid_ori_i] = millis();

          distanceSensor1.startRanging();
          if ((pid_ori_i) < 18)
          {
            distance1_data[pid_ori_i] = (float)distanceSensor1.getDistance();
          }
          // sum_dist += (float)distanceSensor1.getDistance();
          distanceSensor1.clearInterrupt();
          distanceSensor1.stopRanging();

          // roll
          myICM.getAGMT();

          unsigned long current_tm = millis();

          float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
          dt = (current_tm - last_time) / 1000.;
          roll_g = roll_g + myICM.gyrX() * dt;

          // pitch
          float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
          dt = (current_tm - last_time) / 1000.;
          pitch_g = pitch_g + myICM.gyrY() * dt;

          // yaw
          dt = (current_tm - last_time) / 1000.;
          last_time = current_tm;
          yaw_g = yaw_g + myICM.gyrZ() * dt;
          gyro_yaw_raw[pid_ori_i] = yaw_g;
          target_angles[pid_ori_i] = (float)target_angle; // Store current target

          // lpf
          if (pid_ori_i == 0)
          {
            acc_pitch_lpf[0] = acc_pitch_raw[0];
            comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];
            acc_roll_lpf[0] = acc_roll_raw[0];
            comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];
          }
          else
          {
            // lpf pitch
            float pitch_raw_curr = acc_pitch_raw[pid_ori_i];
            acc_pitch_lpf[pid_ori_i] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[pid_ori_i - 1];
            acc_pitch_lpf[pid_ori_i - 1] = acc_pitch_lpf[pid_ori_i];

            comp_pitch[pid_ori_i] = (1 - alpha) * gyro_pitch_raw[pid_ori_i] + alpha * acc_pitch_lpf[pid_ori_i];

            // lpf roll
            float roll_raw_curr = acc_roll_raw[pid_ori_i];
            acc_roll_lpf[pid_ori_i] = alpha * roll_raw_curr + (pid_ori_i - alpha) * acc_roll_lpf[pid_ori_i - 1];
            acc_roll_lpf[pid_ori_i - 1] = acc_roll_lpf[pid_ori_i];

            comp_roll[pid_ori_i] = (1 - alpha) * gyro_roll_raw[pid_ori_i] + alpha * acc_roll_lpf[pid_ori_i];
          }
          pid_ori_i++;
          j++;
        }
        delay(100);
      }
    }

    Serial.print("PID ORI: ");
    Serial.println(pid_ori_i);
    for (int i = 0; i < pid_ori_i; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(distance1_data[i]);
      Serial.print("Sent: ");
      Serial.println(distance1_data[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }

    break;
  }

  case GET_PID_DATA_ORI:
  {
    for (int i = 0; i < num_vars_data_collection; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(pid_times[i]);
      tx_estring_value.append(",");

      tx_estring_value.append(pid_current_angles[i]);
      tx_estring_value.append(",");

      tx_estring_value.append(pid_speeds[i]);
      tx_estring_value.append(",");

      tx_estring_value.append(pwm_data[i]);

      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case FIND_STEADY_STATE:
  {
    set_up_tof();
    int st = millis();
    unsigned long Tdiff = 0;
    distanceSensor1.startRanging();
    distanceSensor1.setIntermeasurementPeriod(20);
    distanceSensor1.setTimingBudgetInMs(20);
    while (Tdiff < 3000)
    {
      Tdiff = millis() - st;
      drive(1, 150);
      if (distanceSensor1.checkForDataReady())
      {
        distance1_data[i] = distanceSensor1.getDistance();
        times[i] = millis();
        distanceSensor1.clearInterrupt();
        i++;
      }
    }
    drive(0, 0);

    analogWrite(motorR1, 0);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, 0);
    delay(2000);

    distanceSensor1.stopRanging();

    for (int i = 0; i < num_data_msgs; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[i]);
      tx_estring_value.append(",");
      tx_estring_value.append(distance1_data[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case GET_KF_DATA:
  {
    for (int j = 0; j < num_vars_data_collection; j++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[j]);
      tx_estring_value.append(",");
      tx_estring_value.append(distance1_data[j]);
      tx_estring_value.append(",");
      tx_estring_value.append(kf_distance_data[j]);
      tx_estring_value.append(",");
      tx_estring_value.append(pwm_data[j]);
      // Serial.println(tx_estring_value.c_str());
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case STUNT:
  {
    // tof();
    Serial.println("Entering stunt loop");
    pid_on = false;
    // drive(0,0);
    int distance;
    int j = 0;
    for (int i = 0; i < 2000; i++)
    {
      distanceSensor1.startRanging();
      if (distanceSensor1.checkForDataReady())
      {
        distance = distanceSensor1.getDistance();
        distance1_data[j] = distance;
        pwm_data[j] = 0;
        times[j] = (float)millis();
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        j++;
      }
    }
    drive(1, 255);

    for (int i = 0; i < 600; i++)
    {
      distanceSensor1.startRanging();
      if (distanceSensor1.checkForDataReady())
      {
        distance = distanceSensor1.getDistance();
        distance1_data[j] = distance;
        pwm_data[j] = 255;
        times[j] = (float)millis();
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        j++;
      }
    }
    // drive(0,0);
    drive(-1, 255);

    for (int i = 0; i < 900; i++)
    {
      distanceSensor1.startRanging();
      if (distanceSensor1.checkForDataReady())
      {
        distance = distanceSensor1.getDistance();
        distance1_data[j] = distance;
        pwm_data[j] = 255;
        times[j] = (float)millis();
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        j++;
      }
    }

    drive(0, 0);

    for (int k = 0; k < 500; k++)
    {
      tx_estring_value.clear();
      tx_estring_value.append(times[k]);
      tx_estring_value.append(",");
      tx_estring_value.append(distance1_data[k]);
      tx_estring_value.append(",");
      tx_estring_value.append(pwm_data[k]);
      // Serial.println(tx_estring_value.c_str());
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case NAVIGATION:
  {
    set_up_tof();
    max_duration = 50;
    max_straight = 500;
    max_angle = 200;
    first_time = true;
    do_navigation = true;

    float waypoints[][2] = {
        {-4.0, -3.0},
        {-2.0, -1.0},
        {1.0, -1.0},
        {2.0, -3.0},
        {5.0, -3.0},
        {5.0, -2.0},
        {5.0, 3.0},
        {0.0, 3.0},
        {0.0, 0.0}};

    for (int i = 1; i < 9; i++)
    {
      target_x = waypoints[i][0];
      target_y = waypoints[i][1];

      start_x = waypoints[i - 1][0];
      start_y = waypoints[i - 1][1];

      do_navigation = true;

      float dx = (target_x - start_x) * 304.8;
      float dy = (target_y - start_y) * 304.8;

      // int delay_time = delay_arrays[i];
      get_to_waypoint(dx, dy);
    }
    break;
  }

  default:
    Serial.print("Invalid Command Type: ");
    Serial.println(cmd_type);
    break;
  }
}

void setup()
{
  pinMode(XSHUT, INPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);

  Serial.begin(115200);

  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on central devices
  tx_characteristic_float.writeValue(0.0);

  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again..");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  set_up_tof();

  ////DMP initialization////

  bool DMP_success = true;
  // Initialize the DMP
  DMP_success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP Game Rotation Vector sensor
  DMP_success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
  // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  DMP_success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok);
  // Enable the FIFO queue
  DMP_success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  DMP_success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  DMP_success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  DMP_success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  // Check success
  if (!DMP_success)
  {
    Serial.println("Enabling DMP failed!");
    while (1)
    {
      // Freeze
    }
  }
  ////DMP initialization////

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  // blink upon start up
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

void write_data()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {

    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000)
    {
      tx_float_value = 0;
    }

    previousMillis = currentMillis;
  }
}

void read_data()
{
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written())
  {
    handle_command();
  }
}

// void record_tof_data(int i)
// {
// if (i < num_data_msgs)
// {
// Serial.println("reading tof data");
// distanceSensor1.startRanging();
// distance1_data[i] = distanceSensor1.getDistance();
// distanceSensor1.clearInterrupt();
// distanceSensor1.stopRanging();

// distanceSensor2.startRanging();
// distance2_data[i] = distanceSensor2.getDistance();
// distanceSensor2.clearInterrupt();
// distanceSensor2.stopRanging();
// }
// }

// void record_imu_data(int i)
// {
// if (i < num_data_msgs && myICM.dataReady())
// {
// float yaw_g = 0, roll_g = 0, pitch_g = 0, dt = 0;
// unsigned long last_time = millis();
// if (i != 0)
// {
// last_time = times[i - 1];
// }
// const float alpha = 0.0735;
// // roll
// myICM.getAGMT();

// float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
// dt = (millis() - last_time) / 1000.;
// last_time = millis();
// roll_g = roll_g + myICM.gyrX() * dt;

// // pitch
// float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
// dt = (millis() - last_time) / 1000.;
// last_time = millis();
// pitch_g = pitch_g + myICM.gyrY() * dt;

// // yaw
// dt = (millis() - last_time) / 1000.;
// last_time = millis();
// yaw_g = yaw_g + myICM.gyrZ() * dt;
// gyro_yaw_raw[i] = yaw_g;

// // lpf
// if (i == 0)
// {
// acc_pitch_lpf[0] = acc_pitch_raw[0];
// comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];
// acc_roll_lpf[0] = acc_roll_raw[0];
// comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];
// }
// else
// {
// // lpf pitch
// float pitch_raw_curr = acc_pitch_raw[i];
// acc_pitch_lpf[i] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[i - 1];
// acc_pitch_lpf[i - 1] = acc_pitch_lpf[i];

// comp_pitch[i] = (1 - alpha) * gyro_pitch_raw[i] + alpha * acc_pitch_lpf[i];

// // lpf roll
// float roll_raw_curr = acc_roll_raw[i];
// acc_roll_lpf[i] = alpha * roll_raw_curr + (i - alpha) * acc_roll_lpf[i - 1];
// acc_roll_lpf[i - 1] = acc_roll_lpf[i];

// comp_roll[i] = (1 - alpha) * gyro_roll_raw[i] + alpha * acc_roll_lpf[i];
// }
// Serial.print("pitch: ");
// Serial.println(comp_pitch[i]);
// Serial.print("roll: ");
// Serial.println(comp_roll[i]);
// Serial.print("yaw: ");
// Serial.println(gyro_yaw_raw[i]);
// }
// }

// int pid(float dist, float targetDist)
// {
//     float currDist = dist;
//     float error = dist - targetDist;

//     // Proportional Control
//     float pwm = Kp * error;

//     if (pwm > 0)
//     {
//         if (pwm > max_speed)
//             pwm = max_speed;

//         return pwm;
//     }
//     else if (pwm < 0)
//     {
//         if (pwm < -max_speed)
//             pwm = -max_speed;

//         return pwm;
//     }

//     return pwm;
// }

// int pid(float dist, float targetDist)
// {
//     float currDist = dist;
//     float error = dist - targetDist;

//     float p_term = Kp * error;

//     float dt = 98;
//     error_sum += error * dt;

//     // float i_term = Ki * error_sum;

//     float pwm = p_term; // + i_term;

//     // if (i_term > 200)
//     // {
//     //     i_term = 200;
//     // }
//     // else if (i_term < -200)
//     // {
//     //     i_term = -200;
//     // }

//     if (pwm > 0)
//     {
//         if (pwm > max_speed)
//             pwm = max_speed;

//         return pwm;
//     }
//     else if (pwm < 0)
//     {
//         if (pwm < -max_speed)
//             pwm = -max_speed;

//         return pwm;
//     }

//     return pwm;
// }

void get_to_waypoint(float dx, float dy)
{
  if (do_navigation)
  {
    // Orientation control 1
    float target_theta = atan2(-dy, dx) * (180.0 / PI);
    Serial.print(target_theta);

    float distance_to_move = sqrt(dx * dx + dy * dy);

    error_ori = 0;
    prev_error_ori = 0;
    error_sum_ori = 0;

    int count = 0;
    // target_theta -= 10;
    while (count < max_angle && target_theta != 0)
    {
      pid_ori(target_theta); // Rotate to align with target
      count += 1;
    }

    // unsigned long start_time = millis();
    // while (millis() - start_time < delay_time)
    // {
    //   drive(1, 150);
    // }
    // drive(0, 0);

    // Position control
    float distances = 0;
    for (int i = 0; i < 50; i++)
    {
      distanceSensor1.startRanging();
      distances += distanceSensor1.getDistance();
      distanceSensor1.stopRanging();
    }

    distances = distances / 50.0;
    pid_i = 0;
    start_time = (float)millis();
    error_sum = 0;

    int pwm;
    float target_dist = distances - distance_to_move;

    Serial.print("Target Distance: ");
    Serial.println(target_dist);

    // drive(1, 100);
    // delay(delay_time);
    count = 0;
    // drive(0, 0);
    while (count < max_straight)
    {
      distanceSensor1.startRanging();
      float curr_pos = distanceSensor1.getDistance();
      pwm = pid(curr_pos, target_dist);
      Serial.println(pwm);
      distanceSensor1.stopRanging();
      count += 1;
    }

    // Step 3: Reset to 0

    error_ori = 0;
    prev_error_ori = 0;
    error_sum_ori = 0;

    count = 0;
    target_theta = 0.0;
    while (count < max_angle)
    {
      pid_ori(target_theta); // Rotate to align with target
      count += 1;
    }

    drive(0, 0);
    do_navigation = false;
    delay(500);
  }
}

int pid(float dist, float targetDist)
{
  float error = dist - targetDist;
  float dt = 0.098;

  float p_term = Kp * error;

  error_sum += error * dt;
  error_sum = constrain(error_sum, -100, 100);
  float i_term = Ki * error_sum;

  float d_term = Kd * (error - previous_error) / dt;

  float pwm = p_term + i_term + d_term;

  pwm = constrain(pwm, -max_speed, max_speed);

  previous_error = error;
  if (pwm > 0)
  {
    drive(1, pwm);
  }
  else if (pwm < -20)
  {
    drive(-1, abs(pwm));
  }
  else
  {
    drive(0, 0);
  }

  return pwm;
}

int pid_ori(float target_angle)
{
  current_time = millis();
  float pid_dt = (current_time - prev_time) / 1000;
  prev_time = current_time;

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

      current_angle = quaternion_to_yaw(q1, q2, q3);
    }
  }

  //  while (target_angle > 180)
  //  target_angle -= 360;
  //  while (target_angle < -180)
  //  target_angle += 360;

  // Calculate error
  error_ori = target_angle - current_angle;
  prev_error_ori = error_ori;

  // Handle the +/- 180 degree boundary crossing
  // if (abs(error_ori) > 180.0)
  // {
  // // If error is larger than 180 degrees, it's shorter to go the other way around
  // error_ori += (reciprocal_target - current_angle) / abs(reciprocal_target - current_angle) * 360.0;
  // }
  if (error_ori > 180)
  {
    error_ori -= 360;
  }
  else if (error_ori < -180)
  {
    error_ori += 360;
  }
  //  Serial.print("error: ");
  //  Serial.println(error_ori);

  // Calculate P term
  float p_term = Kp_ori * error_ori;

  // Calculate I term with anti-windup
  error_sum_ori += error_ori * pid_dt;
  error_sum_ori = constrain(error_sum_ori, -100, 100); // Prevent integral windup
  float i_term = Ki_ori * error_sum_ori;

  // Calculate D term
  float d_term = Kd_ori * (error_ori - prev_error_ori) / pid_dt;

  // Calculate speed control signal
  pid_speed = (int)(p_term + i_term + d_term);
  pid_speed = constrain(pid_speed, -255, 255);

  if (pid_speed > 0 && pid_speed < 90)
  {
    pid_speed = 90;
  }
  else if (pid_speed < 0 && pid_speed > -90)
  {
    pid_speed = -90;
  }
  //  Serial.print("pid speed: ");
  //  Serial.println(pid_speed);

  if (abs(error_ori) < pid_ori_error_threshold / 2)
  {
    drive(0, 0); // Stop if we're close enough to target
  }
  else if (pid_speed > 0)
  {
    // drive(-2, abs(pid_speed) * 1.5); // Turn left
    left(abs(pid_speed * 1.3));
  }
  else
  {
    // drive(2, abs(pid_speed)); // Turn right
    right(abs(pid_speed * 1.3));
  }

  return pid_speed;
}

////Function to convert quaternions to yaw angle///
double quaternion_to_yaw(double q1, double q2, double q3)
{
  double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
  double qw = q0;
  double qx = q2;
  double qy = q1;
  double qz = -q3;
  // yaw (z-axis rotation)
  double t3 = +2.0 * (qw * qz + qx * qy);
  double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
  double yaw = atan2(t3, t4) * 180.0 / PI;
  return yaw;
}

//// PWM control functions////
void left(int pwm) // right forward, left backward
{
  analogWrite(motorR1, pwm);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, pwm * calibration_factor);
}

void right(int pwm) // left Forward, right backward
{
  analogWrite(motorR1, 0);
  analogWrite(motorR2, pwm);
  analogWrite(motorL1, pwm * calibration_factor);
  analogWrite(motorL2, 0);
}

void drive(int direction, int pwm)
{
  if (direction == 1)
  { // forward
    analogWrite(motorR1, pwm);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, pwm * calibration_factor);
    analogWrite(motorL2, 0);
  }
  else if (direction == -1)
  { // backward
    analogWrite(motorR1, 0);
    analogWrite(motorR2, pwm);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, pwm * calibration_factor);
  }
  else
  {
    analogWrite(motorR1, 0);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, 0);
  }
}

// void kf(float &current_speed, float &current_dist, float measurement)
// {
//   Matrix<2, 1> mu_p = Ad * x + Bd * current_speed;
//   Matrix<2, 2> sigma_p = Ad * (sigma * ~Ad) + sig_u;

//   Matrix<1, 1> sigma_m = C * (sigma_p * ~C) + sig_z;

//   Invert(sigma_m);
//   Matrix<2, 1> kkf_gain = sigma_p * (~C * sigma_m);

//   Matrix<1, 1> y_m = {measurement - (C * mu_p)(0, 0)};

//   x = mu_p + kkf_gain * y_m;

//   sigma = (I - kkf_gain * C) * sigma_p;

//   current_dist = x(0, 0);
//   current_speed = x(1, 0);
// }

void loop()
{
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central)
  {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected())
    {
      write_data();
      if (pid_on)
      {
        int curr_distance;
        int pwm;

        if (distanceSensor1.checkForDataReady())
        {
          curr_distance = distanceSensor1.getDistance();
          Serial.println(curr_distance);
          distanceSensor1.clearInterrupt();
          distanceSensor1.stopRanging();
          distanceSensor1.startRanging();

          pwm = pid(curr_distance, 304.8);
          Serial.println(pwm);

          // if(i < 1000) {
          //   dist_pid[i] = curr_distance;
          //   kp[i] = pwm;
          //   time_pid[i] = (float)millis();
          // }
        }
        if (pwm > 40)
        {
          drive(1, pwm);
        }
        else if (pwm < -40)
        {
          drive(-1, abs(pwm));
        }
        else
        {
          drive(0, 0);
        }
        i++;
      }
      else
      {
        drive(0, 0);
      }

      // Send data
      read_data();
    }

    drive(0, 0);

    Serial.println("Disconnected");
  }
}

// #include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
// #include <math.h>

// #include "BLECStringCharacteristic.h"
// #include "EString.h"
// #include "RobotCommand.h"
// #include <ArduinoBLE.h>
// #include "SparkFun_VL53L1X.h"
// #include <BasicLinearAlgebra.h> //Use this library to work with matrices

// using namespace BLA; // To declare a matrix

// #define SERIAL_PORT Serial
// #define AD0_VAL 1 // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
// #define blinkPin LED_BUILTIN

// //////////// BLE UUIDs ////////////
// #define BLE_UUID_TEST_SERVICE "2bb357d9-bff9-49ae-92fe-7935d8da4d69"

// #define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

// #define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
// #define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
// //////////// BLE UUIDs ////////////

// //////////// Global Variables ////////////
// BLEService testService(BLE_UUID_TEST_SERVICE);

// BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

// BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
// BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// // RX
// RobotCommand robot_cmd(":|");

// // TX
// EString tx_estring_value;
// float tx_float_value = 0.0;

// long interval = 500;
// static long previousMillis = 0;
// unsigned long currentMillis = 0;

// #define calib 0.78

// //////////// IMU/TOF Constants /////////////

// ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// #define XSHUT 8
// #define ADDRESS 0x30

// SFEVL53L1X distanceSensor1;
// SFEVL53L1X distanceSensor2(Wire, XSHUT);

// //////////// KALMAN FILTERING /////////////

// // KF Variables
// bool run_pid_kf;
// bool last_distance_valid = false;

// float kf_speed;
// float kf_distance;
// float d = 0.4347826086956522;  // drag
// float m = 0.22658842534082704; // momentum
// float kf;

// // A, B, C matrices
// Matrix<2, 2> A = {0, 1,
//                   0, -d / m};

// Matrix<2, 1> B = {0, 1 / m};

// Matrix<1, 2> C = {1, 0};

// // Discretize A & B
// float delta_t = 0.06;

// Matrix<2, 2> Id = {1, 0,
//                    0, 1};

// Matrix<2, 2> Ad = {1, 0.06,
//                    0, 0.88487075};

// Matrix<2, 1> Bd = {0, 0.26479729};

// // Process and measurement noise
// Matrix<2, 2> sigma_u = {1666.66666667, 0,
//                         0, 1666.66666667};

// Matrix<1, 1> sigma_z = {100};

// // initial state covariance
// Matrix<2, 2> sigma = {400, 0,
//                       0, 100};

// // initial state mean
// Matrix<2, 1> x = {3774, 0};

// //////////// PID CONTROL ////////////

// float time_pid[1000];
// float kp[1000];
// float ki[1000];
// float dist_pid[1000];
// float pwm_pid[1000];
// float kf_dist[1000];

// bool start_pid = false;

// float start_time = 0;
// int i = 0;
// float sum_error = 0;
// float KP = 0.15;
// float KI = 0.00008;
// float KD = 90.0;
// float dt;
// float target = 304.8;
// float prev_error_pid = 0;

// //////////// ORIENTATION CONTROL ////////////

// int time_ori[1000];
// int pwm_ori[1000];
// float current_angs[1000];
// float errors[1000];
// float yaw_ori[1000];

// bool start_ori = false;
// int j = 0;

// float start_time_ori;
// float prev_time_ori = 0;
// float current_time_ori = 0;

// float current_angle = 0;
// float error_ori = 0;
// float error_d = 0;
// float error_i = 0;
// int pwm;
// float ori_dt;

// float prev_error = 0;

// float KP_ori = 1.5;
// float KI_ori = 0.01;
// float KD_ori = 0.0;

// float yaw = 0;
// float target_angle;

// volatile unsigned int send = 0;

// //////////// KALMAN FILTER /////////////

// float tof_data[1000];
// int pwm_data[1000];
// float time_data[1000];

// bool move_car = false;

// //////////// STUNT DATA /////////////

// const unsigned int num_data = 500;
// int time_stunt_for[num_data];
// int time_stunt_back[num_data];
// int tof_stunt_for[num_data];
// int tof_stunt_back[num_data];

// //////////// MAPPING /////////////

// int distance_readings[1000];

// float acc_pitch_lpf[1000];
// float acc_pitch_raw[1000];
// float comp_pitch[1000];
// float gyro_pitch_raw[1000];
// float acc_roll_lpf[1000];
// float acc_roll_raw[1000];
// float comp_roll[1000];
// float gyro_roll_raw[1000];
// float gyro_yaw_raw[1000];

// //////////// COMMAND TYPES ////////////

// enum CommandTypes
// {
//   PING,
//   START_SEND,
//   STOP_SEND,
//   START_PID,
//   STOP_PID,
//   SEND_PID_DATA,
//   START_PID_ORI,
//   STOP_PID_ORI,
//   SEND_ORI_DATA,
//   CAR_FORWARDS,
//   STOP_CAR,
//   GET_TOF_DATA,
//   DO_FLIP,
//   MAPPING,
//   START_SPIN,
//   STOP_SPIN,
// };

// void set_up_tof_sensors()
// {
//   Serial.println("VL53L1X Qwiic Test");

//   // set up tof sensors
//   digitalWrite(XSHUT, LOW);
//   distanceSensor1.setI2CAddress(ADDRESS);
//   Serial.print("Distance Sensor 1 Address: 0x");
//   Serial.println(distanceSensor1.getI2CAddress(), HEX);

//   if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
//   {
//     Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
//     while (1)
//       ;
//   }

//   digitalWrite(XSHUT, HIGH);
//   Serial.print("Distance Sensor 2 Address: 0x");
//   Serial.println(distanceSensor1.getI2CAddress(), HEX);

//   if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
//   {
//     Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
//     while (1)
//       ;
//   }
//   Serial.println("Sensors 1 and 2 online!");
// }

// void handle_command()
// {
//   // Set the command string from the characteristic value
//   robot_cmd.set_cmd_string(rx_characteristic_string.value(),
//                            rx_characteristic_string.valueLength());

//   bool success;
//   int cmd_type = -1;

//   // Get robot command type (an integer)
//   /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
//    * since it uses strtok internally (refer RobotCommand.h and
//    * https://www.cplusplus.com/reference/cstring/strtok/)
//    */
//   success = robot_cmd.get_command_type(cmd_type);

//   // Check if the last tokenization was successful and return if failed
//   if (!success)
//   {
//     return;
//   }

//   // Handle the command type accordingly
//   switch (cmd_type)
//   {

//   case START_SEND:
//     send = 1;
//     break;

//   case STOP_SEND:
//     send = 0;
//     break;

//   /*
//     CASE START PID
//   */
//   case START_PID:
//   {
//     set_up_tof_sensors();

//     distanceSensor1.setDistanceModeLong();
//     distanceSensor1.startRanging();

//     // // initialization for Kp, Ki, Kd
//     // success = robot_cmd.get_next_value(KP);
//     // if (!success) {
//     //   return;
//     // }

//     // success = robot_cmd.get_next_value(KI);
//     // if (!success) {
//     //   return;
//     // }

//     // success = robot_cmd.get_next_value(KD);
//     // if (!success) {
//     //   return;
//     // }

//     // tx_estring_value.clear();

//     Serial.print("KP: ");
//     Serial.println(KP);

//     // Serial.print("KI: ");
//     // Serial.println(KI);

//     start_pid = true;

//     i = 0;
//     start_time = (float)millis();
//     sum_error = 0;
//     dt = 0.060;
//   }
//   break;

//   /*
//     CASE STOP PID
//   */
//   case STOP_PID:
//   {
//     start_pid = false;
//     break;
//   }

//   /*
//     CASE SEND PID DATA
//   */
//   case SEND_PID_DATA:
//     for (int i = 0; i < 1000; i++)
//     {
//       tx_estring_value.clear();
//       tx_estring_value.append(time_pid[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(dist_pid[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(pwm_pid[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(kf_dist[i]);
//       tx_characteristic_string.writeValue(tx_estring_value.c_str());
//     }
//     break;

//   /*
//     CASE START ORI PID
//   */
//   case START_PID_ORI:

//     // initialization for Kp, Ki, Kd, Target distance
//     success = robot_cmd.get_next_value(KP_ori);
//     if (!success)
//     {
//       return;
//     }

//     success = robot_cmd.get_next_value(KI_ori);
//     if (!success)
//     {
//       return;
//     }

//     success = robot_cmd.get_next_value(KD_ori);
//     if (!success)
//     {
//       return;
//     }

//     success = robot_cmd.get_next_value(target_angle);
//     if (!success)
//     {
//       return;
//     }

//     tx_estring_value.clear();

//     start_ori = true;

//     j = 0;
//     prev_time_ori = (float)millis();

//     break;

//   /*
//     CASE STOP ORI PID
//   */
//   case STOP_PID_ORI:
//     tx_estring_value.clear();
//     start_ori = false;
//     break;

//   /*
//     CASE SEND ORI PID DATA
//   */
//   case SEND_ORI_DATA:
//     for (int i = 0; i < 1000; i++)
//     {
//       tx_estring_value.clear();
//       tx_estring_value.append(time_ori[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(current_angs[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(pwm_ori[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(errors[i]);
//       tx_estring_value.append("*");
//       tx_characteristic_string.writeValue(tx_estring_value.c_str());
//     }
//     break;

//   case CAR_FORWARDS:

//     set_up_tof_sensors();

//     distanceSensor1.setDistanceModeLong();
//     distanceSensor1.startRanging();
//     move_car = true;
//     i = 0;
//     break;

//   case STOP_CAR:
//     analogWrite(16, 0);
//     analogWrite(15, 0);
//     analogWrite(7, 0);
//     analogWrite(9, 0);
//     break;

//   case GET_TOF_DATA:

//     set_up_tof_sensors();

//     distanceSensor1.setDistanceModeShort();
//     distanceSensor2.setDistanceModeShort();

//     distanceSensor1.startRanging();
//     distanceSensor2.startRanging();

//     for (int i = 0; i < 1000; i++)
//     {

//       while (!distanceSensor1.checkForDataReady())
//       {
//         delay(1);
//       }
//       Serial.println(distanceSensor1.getDistance());
//       // distance1[i] = distanceSensor1.getDistance();
//       // Serial.print("Distance 1: ");
//       // Serial.print(distance1[i]);

//       // while (!distanceSensor2.checkForDataReady())
//       // {
//       //   delay(1);
//       // }
//       // Serial.println(distanceSensor2.getDistance());
//       // Serial.print(" | Distance 2: ");
//       // Serial.println(distance2[i]);
//     }

//     break;

//   /*
//    * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
//    */
//   case PING:
//     tx_estring_value.clear();
//     tx_estring_value.append("PONG");
//     tx_characteristic_string.writeValue(tx_estring_value.c_str());

//     Serial.print("Sent back: ");
//     Serial.println(tx_estring_value.c_str());

//     break;

//   /*
//    * Do a cool flip !
//    */
//   case DO_FLIP:
//   {
//     set_up_tof_sensors();

//     /////////// actual stunt ///////////
//     Serial.println("Flipping");
//     distanceSensor1.setDistanceModeLong();

//     int index = 0;
//     int i = 0;
//     float start;
//     int curr_distance;

//     unsigned long start_flip = millis();

//     while (millis() - start_flip < 900)
//     {
//       distanceSensor1.startRanging();

//       if (distanceSensor1.checkForDataReady())
//       {
//         time_stunt_for[i] = (float)millis();

//         // move forward
//         analogWrite(16, 0);
//         analogWrite(15, 255);
//         analogWrite(7, 255);
//         analogWrite(9, 0);

//         curr_distance = distanceSensor1.getDistance();
//         tof_stunt_for[i] = curr_distance;
//         distanceSensor1.clearInterrupt();
//         i++;
//       }
//     }

//     unsigned long end_flip = millis();

//     while (millis() - end_flip < 1500)
//     {
//       if (distanceSensor1.checkForDataReady())
//       {

//         time_stunt_back[index] = (float)millis();

//         // move backwards
//         analogWrite(16, 255);
//         analogWrite(15, 0);
//         analogWrite(7, 0);
//         analogWrite(9, 255);

//         curr_distance = distanceSensor1.getDistance();
//         tof_stunt_back[index] = curr_distance;
//         distanceSensor1.clearInterrupt();
//         index++;
//       }
//     }

//     analogWrite(16, 0);
//     analogWrite(15, 0);
//     analogWrite(7, 0);
//     analogWrite(9, 0);

//     for (int j = 0; j < 500; j++)
//     {
//       tx_estring_value.clear();
//       tx_estring_value.append(time_stunt_for[j]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(tof_stunt_for[j]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(time_stunt_back[j]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(tof_stunt_back[j]);
//       tx_characteristic_string.writeValue(tx_estring_value.c_str());
//     }

//     Serial.print("Sent back: ");

//     break;
//   }

//   /*
//     * Do environment MAPPING
//     */
//   case MAPPING:
//   {
//     KP_ori = 0.8;
//     KI_ori = 0.1;
//     KD_ori = 0.0;
//     /////////// set-up tof sensors ///////////
//     set_up_tof_sensors();

//     // Reset orientation PID variables
//     error_i = 0;    // Reset integral term
//     prev_error = 0; // Reset previous error

//     distanceSensor1.setDistanceModeLong();
//     distanceSensor1.startRanging();

//     /////////// mapping stuff here ///////////

//     tx_estring_value.clear();

//     int angle_incr = 24;
//     float angle_error = 5.0;
//     int num_readings = 5;
//     int mapping_index = 0; // Index for storing mapping data

//     // Initialize time
//     prev_time_ori = (float)millis();

//     for (int i = 0; i < 360; i = i + angle_incr)
//     {
//       target_angle = (float)i;
//       Serial.print("Target angle: ");
//       Serial.println(target_angle);

//       do
//       {
//         int pwm_val = runOriController();
//         // Serial.print("Target angle: ");
//         // Serial.print(target_angle);
//         // Serial.print(" Current angle: ");
//         // Serial.println(current_angle);
//         delay(10);
//       } while (!(abs(error_ori) < angle_error));

//       // Stop all motors
//       analogWrite(16, 0);
//       analogWrite(15, 0);
//       analogWrite(7, 0);
//       analogWrite(9, 0);

//       // Serial.println("Reading distances...");
//       delay(10); // Allow time for the robot to settle

//       int j = 0;
//       while (j < num_readings)
//       {
//         // Serial.println("Waiting for sensor");
//         if (distanceSensor1.checkForDataReady())
//         {
//           // Start ranging before getting distance
//           int curr_distance = distanceSensor1.getDistance();

//           // Store distance reading
//           if (mapping_index < 1000)
//           { // Prevent array overflow
//             time_ori[mapping_index] = (float)millis();
//             distance_readings[mapping_index] = curr_distance;
//             // read_imu_data(mapping_index);
//             mapping_index++;
//           }

//           // Clear interrupt and stop ranging
//           distanceSensor1.clearInterrupt();
//           distanceSensor1.stopRanging();
//           distanceSensor1.startRanging();

//           j++;
//         }
//         // delay(50); // Short delay between readings
//       }
//     }

//     // // Send the collected data
//     for (int i = 0; i < mapping_index; i++)
//     {
//       tx_estring_value.clear();
//       tx_estring_value.append(time_ori[i]);
//       tx_estring_value.append("*");
//       tx_estring_value.append(distance_readings[i]);
//       tx_estring_value.append("*");
//       // tx_estring_value.append(yaw_values[i]);
//       tx_characteristic_string.writeValue(tx_estring_value.c_str());
//     }

//     Serial.println("Mapping complete!");
//     break;
//   }

//   case START_SPIN:
//    {float start_time_spin = (float)millis();
//    while(1){
//     move_left(60);
//     delay(2500);
//    }
//    break; }

//   case STOP_SPIN:
//       analogWrite(16, 0);
//       analogWrite(15, 0);
//       analogWrite(7, 0);
//       analogWrite(9, 0);
//    break;

//   /*
//    * The default case may not capture all types of invalid commands.
//    * It is safer to validate the command string on the central device (in python)
//    * before writing to the characteristic.
//    */
//   default:
//     Serial.print("Invalid Command Type: ");
//     Serial.println(cmd_type);
//     break;
//   }
// }

// void setup()
// {
//   Serial.begin(115200);

//   // 15 and 7 is forwards
//   pinMode(15, OUTPUT); // lower motors
//   pinMode(16, OUTPUT);
//   pinMode(7, OUTPUT); // upper motors
//   pinMode(9, OUTPUT);

//   BLE.begin();

//   // Set advertised local name and service
//   BLE.setDeviceName("Artemis BLE");
//   BLE.setLocalName("Artemis BLE");
//   BLE.setAdvertisedService(testService);

//   // Add BLE characteristics
//   testService.addCharacteristic(tx_characteristic_float);
//   testService.addCharacteristic(tx_characteristic_string);
//   testService.addCharacteristic(rx_characteristic_string);

//   // Add BLE service
//   BLE.addService(testService);

//   // Initial values for characteristics
//   // Set initial values to prevent errors when reading for the first time on central devices
//   tx_characteristic_float.writeValue(0.0);

//   Serial.println("Hi");

//   Wire.begin();
//   Wire.setClock(400000);
//   bool initialized = false;

//   while (!initialized)
//   {
//     myICM.begin(Wire, AD0_VAL);
//     Serial.print(F("Initialization of the sensor returned: "));
//     Serial.println(myICM.statusString());
//     if (myICM.status != ICM_20948_Stat_Ok)
//     {
//       Serial.println("Trying again..");
//       delay(500);
//     }
//     else
//     {
//       initialized = true;
//     }
//   }

//   ////////// DMP Set-up ///////////////

//   bool success = true;

//   // Initialize the DMP
//   success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

//   // Enable the DMP Game Rotation Vector sensor
//   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

//   // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
//   // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

//   // Enable the FIFO queue
//   success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

//   // Enable the DMP
//   success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

//   // Reset DMP
//   success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

//   // Reset FIFO
//   success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

//   // Check success
//   if (!success)
//   {
//     Serial.println("Enabling DMP failed!");
//     while (1)
//     {
//       // Freeze
//     }
//   }

//   // Output MAC Address
//   Serial.print("Advertising BLE with MAC: ");
//   Serial.println(BLE.address());

//   BLE.advertise();

//   // blink upon start up
//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(500);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(500);
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(500);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(500);
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(500);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(500);
// }

// void record_imu_data(int i)
// {
//   if (i < 1000)
//   {
//     Serial.println("reading imu data");
//     float yaw_g = 0, roll_g = 0, pitch_g = 0, dt = 0;
//     unsigned long last_time = millis();
//     const float alpha = 0.0735;

//     // roll
//     myICM.getAGMT();
//     float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
//     dt = (millis() - last_time) / 1000.;
//     last_time = millis();
//     roll_g = roll_g + myICM.gyrX() * dt;

//     // pitch
//     float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
//     dt = (millis() - last_time) / 1000.;
//     last_time = millis();
//     pitch_g = pitch_g + myICM.gyrY() * dt;

//     // yaw
//     dt = (millis() - last_time) / 1000.;
//     last_time = millis();
//     yaw_g = yaw_g + myICM.gyrZ() * dt;
//     gyro_yaw_raw[i] = yaw_g;

//     // lpf
//     if (i == 0)
//     {
//       acc_pitch_lpf[0] = acc_pitch_raw[0];
//       comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];
//       acc_roll_lpf[0] = acc_roll_raw[0];
//       comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];
//     }
//     else
//     {
//       // lpf pitch
//       float pitch_raw_curr = acc_pitch_raw[i];
//       acc_pitch_lpf[i] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[i - 1];
//       acc_pitch_lpf[i - 1] = acc_pitch_lpf[i];

//       comp_pitch[i] = (1 - alpha) * gyro_pitch_raw[i] + alpha * acc_pitch_lpf[i];

//       // lpf roll
//       float roll_raw_curr = acc_roll_raw[i];
//       acc_roll_lpf[i] = alpha * roll_raw_curr + (i - alpha) * acc_roll_lpf[i - 1];
//       acc_roll_lpf[i - 1] = acc_roll_lpf[i];

//       comp_roll[i] = (1 - alpha) * gyro_roll_raw[i] + alpha * acc_roll_lpf[i];
//     }
//   }
// }

// void write_data()
// {
//   currentMillis = millis();
//   if (currentMillis - previousMillis > interval)
//   {

//     tx_float_value = tx_float_value + 0.5;
//     tx_characteristic_float.writeValue(tx_float_value);

//     if (tx_float_value > 10000)
//     {
//       tx_float_value = 0;
//     }

//     previousMillis = currentMillis;
//   }
// }

// void read_data()
// {
//   // Query if the characteristic value has been written by another BLE device
//   if (rx_characteristic_string.written())
//   {
//     handle_command();
//   }
// }

// // KF
// float KF(bool run_pid_kf, float u, int distance)
// {
//   // Prediction
//   Matrix<1, 1> u_mat = {u};
//   Matrix<2, 1> mu_p = Ad * x + Bd * u_mat;
//   Matrix<2, 2> sigma_p = Ad * sigma * (~Ad) + sigma_u;

//   if (run_pid_kf)
//   {
//     Matrix<1, 1> sig_m = C * sigma_p * (~C) + sigma_z;
//     Matrix<1, 1> sig_m_inv = sig_m;
//     Invert(sig_m_inv);

//     Matrix<2, 1> kf_gain = sigma_p * (~C) * sig_m_inv;
//     Matrix<1, 1> y_mat = {(float)distance};
//     Matrix<1, 1> y_m = y_mat - C * mu_p;

//     x = mu_p + kf_gain * y_m;
//     sigma = (Id - kf_gain * C) * sigma_p;
//   }
//   else
//   {
//     x = mu_p;
//     sigma = sigma_p;
//   }

//   return x(0, 0);
// }

// /*
//  PID controller
//  Output: New PWM value
// */
// int runPIController(float dist)
// {

//   float curr_distance, error, P, I, D;
//   dt = abs(start_time - float(millis()));
//   start_time = float(millis());
//   curr_distance = dist;
//   error = curr_distance - target;
//   Serial.println(error);
//   // Proportional Control
//   P = KP * error;

//   // Integral Control
//   sum_error = sum_error + (error * dt);
//   I = KI * sum_error;
//   if (I > 100)
//   {
//     I = 100;
//   }
//   else if (I < -100)
//   {
//     I = -100;
//   }

//   // Derivative Control (using prev_err from global)
//   D = KD * ((error - prev_error_pid) / dt);
//   prev_error_pid = error; // Update for next iteration

//   float pwm = P + I + D;
//   if (pwm > 200)
//   {
//     pwm = 200;
//   }
//   else if (pwm < -200)
//   {
//     pwm = -200;
//   }

//   return (int)(pwm);
// }

// double convert_to_yaw(double q1, double q2, double q3)
// {
//   double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
//   double qw = q0;
//   double qx = q2;
//   double qy = q1;
//   double qz = -q3;
//   double t3 = +2.0 * (qw * qz + qx * qy);
//   double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
//   double yaw = atan2(t3, t4) * 180.0 / PI;
//   return yaw;
// }

// /*
//  Orientation PID controller
//  Output: New PWM value
// */////////////////////////////
// int runOriController()
// {
//   // if (millis() < 1000)
//   // {
//   //   analogWrite(16, 0);
//   //   analogWrite(15, 0);
//   //   analogWrite(7, 0);
//   //   analogWrite(9, 0);
//   //   return 0;
//   // }

//   current_time_ori = (float)millis();
//   ori_dt = (current_time_ori - prev_time_ori) / 1000;
//   prev_time_ori = current_time_ori;

//   icm_20948_DMP_data_t data;
//   myICM.readDMPdataFromFIFO(&data);

//   // Is valid data available?
//   if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
//   {
//     // We have asked for GRV data so we should receive Quat6
//     if ((data.header & DMP_header_bitmap_Quat6) > 0)
//     {
//       double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
//       double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
//       double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

//       // Convert the quaternion to Euler angles...
//       current_angle = convert_to_yaw(q1, q2, q3);
//     }
//   }

//   Serial.print("Current angle: ");
//   Serial.print(current_angle);
//   Serial.print("Target angle: ");
//   Serial.println(target_angle);

//   if (target_angle > 180)
//   {
//     target_angle = target_angle - 360;
//   }
//   else if (target_angle < -180)
//   {
//     target_angle = target_angle + 360;
//   }

//   // if (current_angle > 180)
//   // {
//   //   current_angle = current_angle - 360;
//   // }
//   // else if (current_angle < -180)
//   // {
//   //   current_angle = current_angle + 360;
//   // }

//   error_ori = current_angle - target_angle;
//   // Serial.print("Error_ori: ");
//   // Serial.println(error_ori);
//   // travel the shortest route to the expected angle
//   if (error_ori > 180)
//   {
//     error_ori = error_ori - 360;
//   }
//   else if (error_ori < -180)
//   {
//     error_ori = error_ori + 360;
//   }

//   error_d = (error_ori - prev_error) / ori_dt;
//   error_i = error_i + (error_ori * ori_dt);

//   // Serial.print("KP_ori: ");
//   // Serial.println(KP_ori);
//   int p_term = (int)(KP_ori * error_ori);
//   int i_term = (int)(KI_ori * error_i);
//   int d_term = (int)(KD_ori * error_d);

//   // Serial.println(p_term);
//   // Serial.println(i_term);
//   // Serial.println(d_term);

//   int pwm_control = p_term + i_term + d_term;
//   pwm_control = constrain(pwm_control, -255, 255);

//   // Serial.print("PWM: ");
//   // Serial.println(p_term);

//   if (pwm_control > 0)
//   {
//     if(pwm_control < 100)
//       pwm_control = 100;
//     else if (pwm_control > 200)
//       pwm_control = 200;
//     move_left(pwm_control);
//   }
//   else if (pwm_control < 0)
//   {
//     int abs_pwm = -pwm_control; // Take absolute value
//     if(abs_pwm < 100)
//       abs_pwm = 100;
//     else if (abs_pwm > 200)
//       abs_pwm = 200;
//     pwm_control = abs_pwm;
//     move_right(abs_pwm);

//   }
//   else // pwm_control == 0
//   {
//     // Stop all motors
//     analogWrite(16, 0);
//     analogWrite(15, 0);
//     analogWrite(7, 0);
//     analogWrite(9, 0);
//   }

//   Serial.print("Current angle: ");
//   Serial.print(current_angle);
//   Serial.print("Target angle: ");
//   Serial.println(target_angle);

//   prev_error = error_ori;
//   Serial.print("PWM: ");
//   Serial.println(pwm_control);
//   return pwm_control;
// }

// // turn left
// void move_left(int pwm)
// {
//   analogWrite(16, pwm);
//   analogWrite(15, 0);
//   analogWrite(7, pwm);
//   analogWrite(9, 0);
// }

// // turn right
// void move_right(int pwm)
// {
//   analogWrite(16, 0);
//   analogWrite(15, pwm);
//   analogWrite(7, 0);
//   analogWrite(9, pwm);
// }

// void moveForward(int dir, int pwm)
// {
//   // forwards
//   if (dir == 1)
//   {
//     analogWrite(16, 0);
//     analogWrite(15, pwm * calib);
//     analogWrite(7, pwm);
//     analogWrite(9, 0);
//   }
//   // backwards
//   else if (dir == -1)
//   {
//     analogWrite(16, pwm);
//     analogWrite(15, 0);
//     analogWrite(7, 0);
//     analogWrite(9, pwm * calib);
//   }
// }

// void loop()
// {
//   // Listen for connections
//   BLEDevice central = BLE.central();

//   // If a central is connected to the peripheral
//   if (central)
//   {
//     Serial.print("Connected to: ");
//     Serial.println(central.address());

//     // While central is connected
//     while (central.connected())
//     {
//       write_data();
//       read_data();
//       if (start_ori)
//       {
//         pwm = runOriController();
//         if (j < 1000)
//         {
//           time_ori[j] = current_time_ori;
//           pwm_ori[j] = pwm;
//           current_angs[j] = current_angle;
//           errors[j] = error_ori;
//           j++;
//         }
//       }
//       else
//       {
//         analogWrite(16, 0);
//         analogWrite(15, 0);
//         analogWrite(7, 0);
//         analogWrite(9, 0);
//       }
//       // if (move_car)
//       // {
//       //     float curr_distance;
//       //     pwm = 200;
//       //     if (distanceSensor1.checkForDataReady()) {
//       //     curr_distance = distanceSensor1.getDistance();
//       //     Serial.println(curr_distance);
//       //     distanceSensor1.clearInterrupt();
//       //     distanceSensor1.stopRanging();
//       //     distanceSensor1.startRanging();
//       //     moveForward(1, pwm);
//       //     if(i < 1000) {
//       //       time_data[i] = (float)millis();
//       //       tof_data[i] = curr_distance;
//       //       pwm_data[i] = pwm;
//       //       i++;
//       //     }
//       //   }
//       // }
//     }
//     analogWrite(16, 0);
//     analogWrite(15, 0);
//     analogWrite(7, 0);
//     analogWrite(9, 0);

//     Serial.println("Disconnected");
//   }
// }
