#include <I2Cdev.h>
#include "src/MPU6050/MPU6050_9Axis_MotionApps41.h"
#include "src/EEPROM_Utils/EEPROM_Utils.h"
#include "src/IMU_Utils/IMU_Utils.h"

s_module *imus_global;
bool dataFlag = false;
int numIMU = 0;

uint8_t mux_actual=-1;
uint8_t channel_actual=-1;

int maxFreqDMP = 200;
int maxFreqSystem = 500;

udp_data ROS_data;

WiFiClient client;
BluetoothSerial SerialBT;


void setup() {
  Serial.begin(115200);

  // Enabling multiplexer
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH); 
  
  // Configuring Bluetooth
  SerialBT.begin("ESP32 Test Device");
  Serial.println("The device started, now you can pair it with bluetooth!");

  // EEPROM Initialization
  int  ret = EEPROM.begin(512);
  if (ret!=1){
    Serial.println ("error initializing EEPROM");
  }
  delay(1000);  

  // Configure UDP information
  get_udp_data(&ROS_data);
  Serial.println("************************************************************************************"); 
  Serial.println("Stored info");
  print_udp_data(ROS_data);
  Serial.println("************************************************************************************"); 
  getDatafromSerial(&SerialBT);
  get_udp_data(&ROS_data);
  Serial.println("Final info");
  print_udp_data(ROS_data);
  Serial.println("************************************************************************************"); 

  // Connect to WiFi
  setupWiFi(ROS_data);

  
  Wire.begin(33,25); // (SDA,SCL) (33,25) and (  27,14)// please check if you use the single multiplexer or two

  numIMU = countIMUs();
  int divider = (((maxFreqDMP*numIMU)/maxFreqSystem)-1);
  if(numIMU <= 1){
    setDivider(0);
  }else{
    setDivider(divider);
  }
   
  if(numIMU == 0){
    Serial.println("No IMU sensor detected");
  }else{
    Serial.print(numIMU);
    Serial.println(" IMU detected");
  }
  imus_global = new s_module[numIMU];
  
  Serial.println("************************************************************************************"); 
  Wire.setClock(1000000); // super fast mode 1MHZ
  findIMUs(imus_global);
}


/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][ACC X ][ACC Y ][ACC Y ][ACC Z ][ACC Z ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */

void loop(){  
  for (int i = 0;i <numIMU ;i++){
    if(imus_global[i].dmpReady){
      if(imus_global[i].channel!= channel_actual || imus_global[i].mux != mux_actual){
        tcaselect(5,mux_actual);
        mux_actual = imus_global[i].mux;
        channel_actual = imus_global[i].channel;
        tcaselect(channel_actual,mux_actual);
      }

      int res = imus_global[i].sensor.GetCurrentFIFOPacket(imus_global[i].fifoBuffer,imus_global[i].packetSize);
        
      if (res == 1){ // Get the Latest packet 
        imus_global[i].sensor.dmpGetQuaternion(&imus_global[i].q, imus_global[i].fifoBuffer);
        imus_global[i].sensor.dmpGetAccel(&imus_global[i].accel,imus_global[i].fifoBuffer);
        imus_global[i].sensor.dmpGetGyro(&imus_global[i].gyro,imus_global[i].fifoBuffer);
        imus_global[i].sensor.dmpGetMag(&imus_global[i].mag, imus_global[i].fifoBuffer);
        imus_global[i].newFlag = 1;
      }else{
        imus_global[i].newFlag = 0;
      }
    }
  }
  sendData(imus_global,ROS_data,numIMU); 
  
}
