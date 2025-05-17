
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "SparkFun_VL53L1X.h"

#define SERIAL_PORT Serial
#define AD0_VAL   1     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "2bb357d9-bff9-49ae-92fe-7935d8da4d69"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938" 
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
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

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

#define calib 1.2 

///////// Variables for IMU/ToF Labs /////////////
const int num_data = 1000; 
float roll[num_data]; 
float pitch[num_data];

float roll_lpf[num_data]; 
float pitch_lpf[num_data];

double temp_array[1000];
int time_array[1000];

float roll_g_array[num_data]; 
float pitch_g_array[num_data];
float yaw_g_array[num_data]; 

float roll_comp[num_data]; 
float pitch_comp[num_data];
float yaw_comp[num_data];

const float alpha = 0.12; 

volatile unsigned int ind = 0; 
float pitch_main_a[2000]; 
float roll_main_a[2000];

int time_main[2000];

//////////// PID CONTROL ////////////

int time_pid[1000]; 
float kp[1000];
float ki[1000];
float wind_up[1000];
float dist_pid[1000];

bool start_pid = false; 
int start_time_pid = 0; 
int i = 0;
float sum_error = 0; 
float KP = 0;


//////////// ORIENTATION CONTROL ////////////

int time_ori[1000]; 
int pwm_ori[1000];
float current_angs[1000];  
float errors[1000];

float kp_ori[1000];
float ki_ori[1000];
float kd_ori[1000];
float yaw_ori[1000];

float current_angle; 
int start_time_ori;
int prev_time_ori;
bool start_ori = false; 
int j = 0;
int error_ori = 0; 
int error_d = 0; 
int error_i = 0; 
int pwm; 

int ori_dt; 

float current_time_ori = 0; 

float prev_error = 0;

float KP_ori = 0;
float KI_ori = 0;
float KD_ori = 0;

float yaw = 0;
float target = 0; 

volatile unsigned int send = 0; 

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

#define XSHUT 8
#define ADDRESS 0x30

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);

//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS, 
    GET_TIME_LOOP,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    PERFORMANCE,  
    GET_PITCH_DATA, 
    GET_ROLL_DATA, 
    GET_YAW_G_DATA, 
    GET_ROLL_G_DATA, 
    GET_PITCH_G_DATA, 
    SEND_ALL_DATA, 
    START_SEND,
    STOP_SEND, 
    RUN_PID,
    STOP_CAR,
    SEND_PID_DATA, 
    START_PID_ORI, 
    STOP_PID_ORI, 
    SEND_ORI_DATA, 
    
};


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
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;

        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            float float_a, float_b, float_c;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;
            
            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            Serial.print("Three Integers: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);

            break;

        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            tx_estring_value.clear();
            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append("Robot says: ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append("!");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            
            break;

        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        
        /*
         * GET_TIME_MILLIS
         */
        case GET_TIME_MILLIS:
            
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(int(millis()));

            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break; 
        

        /*
         * GET_TIME_LOOP
         */
        case GET_TIME_LOOP:
            
           { 
            unsigned long startTime = millis(); 
            int count = 0; 

            // run loop for 5 seconds 
            while(millis() - startTime  < 5000)
            {
              tx_estring_value.clear();
              tx_estring_value.append("T: ");
              tx_estring_value.append(int(millis()));

              tx_characteristic_string.writeValue(tx_estring_value.c_str());

              count ++; 
            }
            Serial.println("Sent back time data");
            break; }

        /*
         * SEND_TIME_DATA
         */
        case SEND_TIME_DATA:
            
           { 
            unsigned long startTime = millis(); 
            int i = 0; 
            
            // run loop for 5 seconds 
            while(millis() - startTime  < 5000)
            {
              if(i < 1000)
              {
                time_array[i] = (int)millis(); 
                i++; 
              }

            }

            for(int i = 0; i < 1000; i++)
            {
              tx_estring_value.clear();
              tx_estring_value.append("Count "); 
              tx_estring_value.append(i);
              tx_estring_value.append(": ");
              tx_estring_value.append(time_array[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back time data");
            break; 
            
            }

        /*
         * GET_TEMP_READINGS
         */
        case GET_TEMP_READINGS:
            
           { 
            unsigned long startTime = millis(); 
            int i = 0; 
            
            // run loop for 5 seconds 
            while(millis() - startTime  < 5000)
            {
              if(i < 1000)
              {
                temp_array[i] = (float)getTempDegF(); 
                time_array[i] = (int)millis(); 
                i++;   
              }

            }

            for(int j = 0; j < 1000; j++)
            {
              tx_estring_value.clear();
              tx_estring_value.append("Time: "); 
              tx_estring_value.append(time_array[j]);
              tx_estring_value.append(" * ");
              tx_estring_value.append("Temperature: "); 
              tx_estring_value.append(temp_array[j]);
              Serial.println(temp_array[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back data");
            break; 
            
            }

        case PERFORMANCE:

            tx_estring_value.clear();
            char resp[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(resp);
            if (!success)
                return;

            tx_estring_value.clear();
            tx_estring_value.append(resp);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;

        case GET_PITCH_DATA: 

          if(myICM.dataReady())
          {            
            float pitch_a = 0, roll_a = 0;

            for(int i = 1; i < num_data; i++)
            {
              myICM.getAGMT();     
              pitch_a = atan2(myICM.accY(),myICM.accZ()) * 180/M_PI; 
              
              Serial.print(", pitch_a:");
              Serial.println(pitch_a);
              
              pitch[i] = pitch_a;
              time_array[i] = int(millis());

            }

            pitch_lpf[0] = pitch[0]; 
            for(int i = 1; i < num_data; i++)
            {              
              Serial.print(", pitch_a:");
              Serial.println(pitch_a);
              
              pitch_lpf[i] = alpha * pitch[i] + (1 - alpha) * pitch_lpf[i - 1];
              pitch_lpf[i - 1] = pitch_lpf[i];

              time_array[i] = int(millis());

            }


            for(int j = 0; j < num_data; j++)
            {
              tx_estring_value.clear();
              tx_estring_value.append(time_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(pitch[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(pitch_lpf[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back data");
      
        }
          break; 
        

        case GET_ROLL_DATA: 

          if(myICM.dataReady())
          {            
            float pitch_a = 0, roll_a = 0;
            roll_lpf[0] = roll[0]; 
            for(int i = 1; i < num_data; i++)
            {
              myICM.getAGMT();     
              roll_a  = atan2(myICM.accX(),myICM.accZ()) * 180/M_PI;
              
              Serial.print(", roll_a:");
              Serial.println(roll_a);
              
              roll[i] = roll_a; 
              roll_lpf[i] = alpha * roll_a + (1 - alpha) * roll_lpf[i - 1];
              roll_lpf[i - 1] = roll_lpf[i];

              time_array[i] = int(millis());
            }

            for(int j = 0; j < num_data; j++)
            {
              tx_estring_value.clear();
              tx_estring_value.append(time_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(roll[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(roll_lpf[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back data");
      
        }
          break; 
        

        case GET_PITCH_G_DATA: 

          if(myICM.dataReady())
          {               
            float pitch_g = 0, dt = 0;
            float pitch_a = 0; 

            unsigned long last_time = millis();
            
            for(int i = 1; i < num_data; i++)
            {
              myICM.getAGMT(); 
              dt = (millis()-last_time)/1000000.0;   
              last_time = millis(); 
              pitch_g = pitch_g + myICM.gyrX()*dt;;
              pitch_a = atan2(myICM.accY(),myICM.accZ()) * 180/M_PI; 

              Serial.print(", pitch_g:");
              Serial.println(pitch_g);
              
              pitch[i] = pitch_a; 
              pitch_g_array[i] = pitch_g; 
              time_array[i] = int(millis());
            }

            pitch_lpf[0] = pitch[0]; 
            for(int i = 1; i < num_data; i++)
            {                            
              pitch_lpf[i] = alpha * pitch[i] + (1 - alpha) * pitch_lpf[i - 1];
              pitch_lpf[i - 1] = pitch_lpf[i];

            }

            pitch_comp[0] = pitch_g_array[0]; 

            //Complimentary Filter
            for(int i = 1; i < num_data; i++)
            {                            
              dt = (time_array[i] - time_array[i-1])/1000; 
              pitch_comp[i] = (pitch_comp[i] - myICM.gyrY()*dt)*0.9 + pitch_lpf[i]*0.1;
            }
            
            for(int j = 0; j < num_data; j++)
            {
              tx_estring_value.clear();
              tx_estring_value.append(time_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(pitch_g_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(pitch[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(pitch_lpf[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(pitch_comp[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back data");
      
        }
          break; 

        case GET_ROLL_G_DATA: 

          if(myICM.dataReady())
          {               
            float roll_g = 0, dt = 0;
            float roll_a = 0; 

            unsigned long last_time = millis();
            
            for(int i = 1; i < num_data; i++)
            {
              myICM.getAGMT(); 
              dt = (millis()-last_time)/1000000.0;   
              last_time = millis(); 
              roll_g = roll_g + myICM.gyrY()*dt;
              roll_a = atan2(myICM.accY(),myICM.accZ()) * 180/M_PI; 

              Serial.print(", roll_g:");
              Serial.println(roll_g);
              
              roll[i] = roll_a; 
              roll_g_array[i] = roll_g; 
              time_array[i] = int(millis());
            }

            roll_lpf[0] = roll[0]; 
            for(int i = 1; i < num_data; i++)
            {                            
              roll_lpf[i] = alpha * roll[i] + (1 - alpha) * roll_lpf[i - 1];
              roll_lpf[i - 1] = roll_lpf[i];

            }

            roll_comp[0] = roll_g_array[0]; 

            //Complimentary Filter
            for(int i = 1; i < num_data; i++)
            {                            
              dt = (time_array[i] - time_array[i-1])/1000; 
              roll_comp[i] = (roll_comp[i] - myICM.gyrY()*dt)*0.9 + roll_lpf[i]*0.1;
            }
            
            for(int j = 0; j < num_data; j++)
            {
              tx_estring_value.clear();
              tx_estring_value.append(time_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(roll_g_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(roll[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(roll_lpf[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(roll_comp[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back data");
      
        }
          break; 

        
        case GET_YAW_G_DATA: 

          if(myICM.dataReady())
          {               
            float yaw_g = 0, dt = 0;
            unsigned long last_time=millis();

            for(int i = 1; i < num_data; i++)
            {
              myICM.getAGMT(); 
              
              dt = (millis()-last_time)/1000000.0;   
              last_time = millis(); 
              yaw_g = yaw_g + myICM.gyrZ()*dt;
              
              Serial.print(", yaw_g:");
              Serial.println(yaw_g);
              
              yaw_g_array[i] = yaw_g; 
              time_array[i] = int(millis());
            }

            for(int j = 0; j < num_data; j++)
            {
              tx_estring_value.clear();
              tx_estring_value.append(time_array[j]);
              tx_estring_value.append("*");
              tx_estring_value.append(yaw_g_array[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.println("Sent back data");
      
        }
          break; 
        
        case START_SEND: 
        send = 1;
        break; 

        case STOP_SEND: 
        send = 0;
        break; 

        case SEND_ALL_DATA: 

          for(int i = 1; i < 2000; i++)
          {
             if(time_main[i] == 0 || pitch_main_a[i] == 0 || roll_main_a[i] == 0){
              break;
             }
             tx_estring_value.clear();
             tx_estring_value.append(time_main[i]);
             tx_estring_value.append("*");
             tx_estring_value.append(pitch_main_a[i]);
             tx_estring_value.append("*");
             tx_estring_value.append(roll_main_a[i]);
             tx_estring_value.append("*");
             tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
        //   if(myICM.dataReady())
        //   {               
        //     float raw_yaw_g = 0, raw_pitch_g = 0, raw_roll_g = 0; 
        //     float raw_pitch_a = 0, raw_roll_a = 0; 
        //     unsigned long last_time = millis();
        //     float dt = 0;
             
        //     for(int i = 0; i < num_data; i++)
        //     {
        //       myICM.getAGMT(); 
              
        //       dt = (millis()-last_time)/1000000.0;   
        //       last_time = millis(); 

        //       raw_yaw_g = raw_yaw_g + myICM.gyrZ()*dt;
        //       raw_pitch_g = raw_pitch_g + myICM.gyrX()*dt;
        //       raw_roll_g = raw_roll_g + myICM.gyrY()*dt;

        //       raw_pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
        //       raw_roll_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 

        //       roll_g_array[i] = raw_roll_g; 
        //       pitch_g_array[i] = raw_pitch_g; 
        //       yaw_g_array[i] = raw_yaw_g; 

        //       roll[i] = raw_roll_a; 
        //       pitch[i] = raw_pitch_a; 

        //       time_array[i] = int(millis());
        //     }

            

        //     for(int i = 0; i < num_data; i++)
        //     {
        //       myICM.getAGMT(); 
              
        //       dt = (millis()-last_time)/1000000.0;   
        //       last_time = millis(); 

        //       raw_yaw_g = raw_yaw_g + myICM.gyrZ()*dt;
        //       raw_pitch_g = raw_pitch_g + myICM.gyrX()*dt;
        //       raw_roll_g = raw_roll_g + myICM.gyrY()*dt;

        //       raw_pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
        //       raw_roll_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 

        //       roll_g_array[i] = raw_roll_g; 
        //       pitch_g_array[i] = raw_pitch_g; 
        //       yaw_g_array[i] = raw_yaw_g; 

        //       roll[i] = raw_roll_a; 
        //       pitch[i] = raw_pitch_a; 

        //       time_array[i] = int(millis());
        //     }

        //     // LPF filtering
        //     pitch_lpf[0] = pitch[0]; 
        //     roll_lpf[0] = roll[0]; 
        //     for(int i = 1; i < num_data; i++)
        //     {                            
        //       pitch_lpf[i] = alpha * pitch[i] + (1 - alpha) * pitch_lpf[i - 1];
        //       pitch_lpf[i - 1] = pitch_lpf[i];

        //       roll_lpf[i] = alpha * roll[i] + (1 - alpha) * roll_lpf[i - 1];
        //       roll_lpf[i - 1] = roll_lpf[i];

        //     }

        //     roll_comp[0] = roll_g_array[0]; 
        //     pitch_comp[0] = pitch_g_array[0];
        //     //Complimentary Filter
        //     for(int i = 1; i < num_data; i++)
        //     {                            
        //       dt = (time_array[i] - time_array[i-1])/1000; 
        //       roll_comp[i] = (roll_comp[i] - myICM.gyrY() * dt) * (1-alpha) + roll_lpf[i] * alpha;
        //       pitch_comp[i] = (pitch_comp[i] - myICM.gyrY() * dt) * (1-alpha) + pitch_lpf[i] * alpha;
        //       yaw_comp[i] = (pitch_comp[i] - myICM.gyrY() * dt) * (1-alpha);
        //     }

        //     for(int j = 0; j < num_data; j++)
        //     {
        //       tx_estring_value.clear();
        //       tx_estring_value.append(time_array[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(roll[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(pitch[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(yaw_g_array[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(roll_g_array[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(pitch_g_array[j]);
        //       tx_estring_value.append( " | ");
        //       tx_estring_value.append(pitch_lpf[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(roll_lpf[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(roll_comp[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(pitch_comp[j]);
        //       tx_estring_value.append(" | ");
        //       tx_estring_value.append(yaw_comp[j]);
        //       tx_characteristic_string.writeValue(tx_estring_value.c_str());
        //     }

            Serial.println("Sent back data");
      
        // }
          break; 


        case RUN_PID:
          {
             Serial.println("VL53L1X Qwiic Test");

              // set up tof sensors
              digitalWrite(XSHUT, LOW);
              distanceSensor1.setI2CAddress(ADDRESS);
              Serial.print("Distance Sensor 1 Address: 0x");
              Serial.println(distanceSensor1.getI2CAddress(), HEX);

              if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
              {
                Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
                while (1);
              }

              digitalWrite(XSHUT, HIGH);
              Serial.print("Distance Sensor 2 Address: 0x");
              Serial.println(distanceSensor1.getI2CAddress(), HEX);

              if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
              {
                Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
                while (1);
              }
              Serial.println("Sensors 1 and 2 online!");
              
              distanceSensor1.setDistanceModeLong();
              distanceSensor1.startRanging();

              i = 0;
              start_time_pid = (float) millis();
              sum_error = 0; 
              start_pid = true;
              KP = 0.05; 

          }
          break; 
        
        case STOP_CAR:
          {
            start_pid = false;

            break;
          }

        case START_PID_ORI: 
          
          // initialization for Kp, Ki, Kd, Target distance
          success = robot_cmd.get_next_value(KP_ori);
          if (!success) {
            return;
          }
          
          success = robot_cmd.get_next_value(KI_ori);
          if (!success) {
            return;
          }

          success = robot_cmd.get_next_value(target);
          if (!success) {
            return;
          }

          tx_estring_value.clear();

          j = 0;
          start_time_ori = (float) millis();
          
          current_angle = 0; 
          start_ori = true;
          KP_ori = 2; 

        break; 

        case STOP_PID_ORI: 
          tx_estring_value.clear();
          start_ori = false; 
        break; 

        /*
        * Send recorded data
        */
        case SEND_ORI_DATA: 
          for(int i = 0; i < 1000; i++)
          {
            tx_estring_value.clear();
            tx_estring_value.append(time_ori[i]);
            tx_estring_value.append("*");
            tx_estring_value.append(current_angs[i]);
            tx_estring_value.append("*");
            tx_estring_value.append(pwm_ori[i]);
            tx_estring_value.append("*");
            tx_estring_value.append(errors[i]);
            tx_estring_value.append("*");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
        break; 

        /*  
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(14, OUTPUT); // lower motors
    pinMode(15, OUTPUT);
    pinMode(7, OUTPUT); // upper motors
    pinMode(9, OUTPUT); 
    
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

    Serial.println("Hi");

    Wire.begin();
    Wire.setClock(400000);
    bool initialized = false;

    while( !initialized )
    {
      myICM.begin( Wire, AD0_VAL );
      Serial.print( F("Initialization of the sensor returned: ") );
      Serial.println( myICM.statusString() );
      if( myICM.status != ICM_20948_Stat_Ok ){
        Serial.println( "Trying again.." );
        delay(500);
      }
      else {
        initialized = true;
      }
    } 

    ////////// DMP Set-up ///////////////

    bool success = true;

    // Initialize the DMP
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // Enable the DMP Game Rotation Vector sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
    // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO queue
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (!success) {
        Serial.println("Enabling DMP failed!");
        while (1) {
            // Freeze
        }
    }

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
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}


void read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}


void sendOriData()
{

  for ( int i = 0; i < 1000; i++ ) {
      tx_estring_value.clear();
      tx_estring_value.append( time_ori[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( kp_ori[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( ki_ori[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( kd_ori[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( wind_up[i] );
      tx_characteristic_string.writeValue( tx_estring_value.c_str() );
    }
}


void sendPIData()
{

  for ( int i = 0; i < 1000; i++ ) {
      tx_estring_value.clear();
      tx_estring_value.append( time_pid[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( dist_pid[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( kp[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( ki[i] );
      tx_estring_value.append( "*" );
      tx_estring_value.append( wind_up[i] );
      tx_characteristic_string.writeValue( tx_estring_value.c_str() );
    }
}



int runPIController(float dist, float target)
{
  float curr_distance, last_distance, P, I, KI, error;  

  curr_distance = dist;
  error = curr_distance - target;

  //Proportional Control
  P = KP * error;

  if (P > 0)
  {
    if (P > 200)
      P = 200;
  }
  else if (P < 0)
  {
    if(P < -200)
      P = -200;
  }
  
  return P;

}

int runOriController()
{
  unsigned long pid_loop_start = micros();

  current_time_ori = millis();
  ori_dt = current_time_ori - prev_time_ori; 
  prev_time_ori = current_time_ori; 

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  // Is valid data available?
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
      // We have asked for GRV data so we should receive Quat6
      if ((data.header & DMP_header_bitmap_Quat6) > 0) {
          double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
          double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
          double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

          // Convert the quaternion to Euler angles...
          current_angle = convert_to_yaw(q1, q2, q3);
      }
  }

  error_ori = current_angle - target; 
  error_d = (error_ori - prev_error)/ori_dt; 
  error_i = error_i + (error_ori * ori_dt); 

  float pwm_fl = KP_ori * error_ori; 

  pwm = (int)(KP_ori * error_ori + KI_ori * error_i + KD_ori * error_d ); 
  
  if (pwm > 200)
  {
    pwm = 200;
  }

  if (pwm < -200)
  {
    pwm = -200;
  }

  if (pwm > 0 &&  pwm < 40){
    pwm = 40;
  }
  if (pwm > -40 && pwm < 0){
    pwm = -40;
  }

  if (pwm > 0)
  {
    move_left(pwm);
  }
  else if (pwm < 0)
  {
    move_right(-pwm); 
  }

  prev_error = error_ori; 

  // // Calculate and store the loop iteration time
  // unsigned long pid_loop_end = micros();
  // unsigned long pid_loop_time = pid_loop_end - pid_loop_start;
  
  // // Store the timing information - add a new array to store loop times
  // if (j < 1000) {
  //   pid_loop_times[j] = pid_loop_time;  // You'll need to add this array to your global variables
  // }
  
  // // Optionally print the timing info
  // Serial.print("PID Loop Time (us): ");
  // Serial.println(pid_loop_time);
  
  return pwm; 


}

// turn left 
void move_left(int pwm)
{
    analogWrite(14, 200);
    analogWrite(15, 0);
    analogWrite(7, 200);
    analogWrite(9, 0);
}


// turn right
void move_right(int pwm)
{
    analogWrite(14, 0);
    analogWrite(15, 200);
    analogWrite(7, 0);
    analogWrite(9, 200);
}


double convert_to_yaw(double q1, double q2, double q3)
{
  double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
  double qw = q0; 
  double qx = q2;
  double qy = q1;
  double qz = -q3;
  double t3 = +2.0 * (qw * qz + qx * qy);
  double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
  double yaw = atan2(t3, t4) * 180.0 / PI;
  return yaw;
}


void moveForward(int dir, int pwm )
{
    // forwards 
   if (dir == 1)
   {
      analogWrite(14, 0);
      analogWrite(15, pwm);
      analogWrite(7, pwm*calib);
      analogWrite(9, 0);
      
   }
   // backwards 
   else if (dir == -1)
   {
      analogWrite(14, 0);
      analogWrite(15, pwm);
      analogWrite(7, 0);
      analogWrite(9, pwm*calib);
   }
    
}


void loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            write_data();
            if (start_ori)
            {
              runOriController();
              if (j < 1000) {
                time_ori[j] = current_time_ori;
                pwm_ori[j] = pwm; 
                current_angs[j] = current_angle; 
                errors[j] = error_ori; 
                j++; 
              }
            } 
            // if (start_pid)
            // { 
            //   int curr_distance; 
            //   int pwm; 

            //   if (distanceSensor1.checkForDataReady()) {
            //       curr_distance = distanceSensor1.getDistance();
            //       Serial.println(curr_distance); 
            //       distanceSensor1.clearInterrupt();
            //       distanceSensor1.stopRanging();
            //       distanceSensor1.startRanging();

            //       pwm = runPIController(curr_distance, 304.8);
            //       Serial.println(pwm); 

            //       if(i < 1000) {
            //         dist_pid[i] = curr_distance;
            //         kp[i] = pwm;
            //         time_pid[i] = (float)millis();
            //       }
            //   }
            //   if(pwm > 40){
            //       moveForward(1, pwm);
            //   }
            //   else if(pwm < -40){
            //       moveForward(-1, abs(pwm));
            //   }
            //   else
            //   {
            //       analogWrite(14, 0);
            //       analogWrite(15, 0);
            //       analogWrite(7, 0);
            //       analogWrite(9, 0);
            //       sendPIData();
            //   }
            //   i++;
            // }
            // else if (start_ori)
            // {
            //   int pwm; 
            //   float gyrZ;
            //   if (myICM.dataReady()) {
            //       gyrZ = myICM.gyrZ(); 
            //       dt = (float(millis()) - last_time_ori) / 1000;
            //       last_time_ori = millis();
            //       yaw[j] = yaw[j-1] + dt * gyrZ;
            //       Serial.println(gyrZ); 

            //       pwm = runOriController(gyrZ, targetAngle);
            //   }
            // }
            // else if (start_pid == false)
            // {
            //   analogWrite(14, 0);
            //   analogWrite(15, 0);
            //   analogWrite(7, 0);
            //   analogWrite(9, 0);
            //   sendPIData();
            // }
            
            // Send data
            read_data();
        }
        
        analogWrite(14, 0);
        analogWrite(15, 0);
        analogWrite(7, 0);
        analogWrite(9, 0);

        Serial.println("Disconnected");
    }   
}


