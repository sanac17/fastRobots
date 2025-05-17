/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo 
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * This code is beerware; if you see me (or any other SparkFun employee) at the
 * local, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ***************************************************************/
 
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

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

const int num_data = 1000; 
float roll[num_data]; 
float pitch[num_data]; 
int tim[num_data]; 

enum CommandTypes
{
    GET_PITCH_ROLL_DATA, 
};

//////////// Global Variables ////////////


ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

void setup() 
{
  Serial.begin(115200);
  
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
    }else{
      initialized = true;
    }
  }

  /*
  * BLE SET-UP
  */

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

}

void handle_command() 
{
  // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

   float pitch_a, roll_a = 0; 
    success = robot_cmd.get_command_type(cmd_type);
    if (!success) {
        return;
    }

    if (cmd_type == GET_PITCH_ROLL_DATA)
    {
      if(myICM.dataReady())
      {    
        for(int i = 0; i<num_data; i++)
        {
          myICM.getAGMT();     
          pitch_a = atan2(myICM.accY(),myICM.accZ()) * 180/M_PI; 
          roll_a  = atan2(myICM.accX(),myICM.accZ()) * 180/M_PI;
          pitch[i] = pitch_a;
          roll[i] = roll_a; 
          tim[i] = int(millis());
        }

      for(int j = 0; j < 1000; j++)
      {
        tx_estring_value.clear();
        tx_estring_value.append(tim[j]);
        tx_estring_value.append("*");
        tx_estring_value.append(roll[j]);
        tx_estring_value.append("*");
        tx_estring_value.append(pitch[j]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
      }

      Serial.println("Sent back data");
    
      }
    }
    else 
    {
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
    }
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


void loop() {
  
  /* Computation variables */
  float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt =0, pitch = 0, roll = 0, yaw = 0;
  float Xm = 0, Ym =0, Zm = 0, x = 0, y = 0;
  unsigned long last_time = millis();
  double pitch_a_LPF[] = {0, 0};
  const int n =1;

  while(1)
  {
    if(myICM.dataReady())
    {
      myICM.getAGMT();                // The values are only updated when you call 'getAGMT'


      //Slide 20 Accelerometer introduction
     
      //NB: Setup Serialplot Y axis [-2000 2000]
      // Serial.print("lower_limit:");
      // Serial.print(-2500);
      // Serial.print(", upper_limit:");
      // Serial.print(2500);
      // Serial.print(", acc_x:"); 
      // Serial.print("acc_x:");
      // Serial.print( myICM.accX() );
      // Serial.print(", acc_y:");
      // Serial.print( myICM.accY() );
      // Serial.print(", acc_z:");
      // Serial.println( myICM.accZ() );
      

      //Slide 24, accelerometer
      //NB: Setup Serialplot Y axis [-180 180]
        Serial.print("lower_limit:");
        Serial.print(-200);
        Serial.print(", upper_limit:");
        Serial.print(200);

        pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
        roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
        
        Serial.print(", pitch_a:");
        Serial.print(pitch_a);
        Serial.print(", roll_a:");
        Serial.println(roll_a); //FOR THE SECOND DEMO, COMMENT OUT LINE FOR THIRD DEMO
        // Serial.print(roll_a); //FOR THE THIRD DEMO, COMMENT OUT LINE FOR SECOND DEMO
        
  
  
        //Slide 25, LPF
        //Tilt along y-axis
        // const float alpha = 0.02;
        // pitch_a_LPF[n] = alpha*pitch_a + (1-alpha)*pitch_a_LPF[n-1];
        // pitch_a_LPF[n-1] = pitch_a_LPF[n];
        // Serial.print(", pitch_LPF:");
        // Serial.println(pitch_a_LPF[n]);

  /*

      //Slide 32, Gyroscope
      Serial.print("lower_limit:");
      Serial.print(-200);
      Serial.print(", upper_limit:");
      Serial.print(200);
      dt = (micros()-last_time)/1000000.;
      last_time = micros();
      pitch_g = pitch_g + myICM.gyrX()*dt;
      roll_g = roll_g + myICM.gyrY()*dt;
      yaw_g = yaw_g + myICM.gyrZ()*dt;
      Serial.print(", pitch_g:");
      Serial.print(pitch_g);
      Serial.print(", roll_g:");
      Serial.print(roll_g);
      Serial.print(", yaw_g:");
      Serial.print(yaw_g);
      Serial.print(", pitch_a:");
      pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
      Serial.println(pitch_a);


      //Slide 38, Magnetic North
      Xm = myICM.magX();
      Ym = myICM.magY();
      Zm = myICM.magZ();
      Serial.print("lower_limit:");
      Serial.print(-200);
      Serial.print(", upper_limit:");
      Serial.print(200);
      Serial.print(", mag_x:");
      Serial.print(Xm);

      //Slide 39, determine yaw
      yaw = atan2(Xm,Ym)*180/M_PI;
      Serial.print(", yaw:");
      Serial.println(yaw);
*/
    }
  }
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}



void blink(unsigned char no)
{
  //Indicate success
  for(char i=0; i<=no-1; i++)
  {
    digitalWrite(blinkPin, HIGH);
    delay(1000);
    digitalWrite(blinkPin, LOW);
    delay(1000);
  }  
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b( int16_t val ){
  if(val > 0){
    SERIAL_PORT.print(" ");
    if(val < 10000){ SERIAL_PORT.print("0"); }
    if(val < 1000 ){ SERIAL_PORT.print("0"); }
    if(val < 100  ){ SERIAL_PORT.print("0"); }
    if(val < 10   ){ SERIAL_PORT.print("0"); }
  }else{
    SERIAL_PORT.print("-");
    if(abs(val) < 10000){ SERIAL_PORT.print("0"); }
    if(abs(val) < 1000 ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 100  ){ SERIAL_PORT.print("0"); }
    if(abs(val) < 10   ){ SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b( agmt.acc.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.acc.axes.z );
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b( agmt.gyr.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.gyr.axes.z );
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b( agmt.mag.axes.x );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.y );
  SERIAL_PORT.print(", ");
  printPaddedInt16b( agmt.mag.axes.z );
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b( agmt.tmp.val );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    SERIAL_PORT.print("-");
  }else{
    SERIAL_PORT.print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      SERIAL_PORT.print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    SERIAL_PORT.print(-val, decimals);
  }else{
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT( ICM_20948_AGMT_t agmt){
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat( myICM.accX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.accZ(), 5, 2 );
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat( myICM.gyrX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.gyrZ(), 5, 2 );
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat( myICM.magX(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magY(), 5, 2 );
  SERIAL_PORT.print(", ");
  printFormattedFloat( myICM.magZ(), 5, 2 );
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat( myICM.temp(), 5, 2 );
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
