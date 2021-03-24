/* Typical pin layout used:
   -----------------------------------------------------------------------------------------
               MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
               Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
   Signal      Pin          Pin           Pin       Pin        Pin              Pin
   -----------------------------------------------------------------------------------------
   RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
   SPI SS      SDA(SS)      10            53        D10        10               10
   SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
   SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
   SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
*/

#include <SPI.h>
#include <MFRC522.h>
#include <ros.h>
#include <arduino_msgs/rfiduid.h>
#include <std_msgs/Bool.h>


#define SS_PIN 10
#define RST_PIN 9

// initialize rfid sensor on ports SS_PIN and RST_PIN
MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

// variable to save last seen UID
byte nuidPICC[4];

// ros node handle
ros::NodeHandle n;

void get_rfid_uid(const arduino_msgs::rfiduid::Request& req, arduino_msgs::rfiduid::Response& resp) {
  // prepare buffer for wake up
  byte buffer [2];
  byte b = 2;

  // wake up receiever
  MFRC522::StatusCode stat = rfid.PICC_WakeupA(buffer, &b);

  if (stat == MFRC522::STATUS_OK) {
    // if something is present  
    rfid.PICC_ReadCardSerial();

    // write uid to message and send it back
    resp.b0 = rfid.uid.uidByte[0];
    resp.b1 = rfid.uid.uidByte[1];
    resp.b2 = rfid.uid.uidByte[2];
    resp.b3 = rfid.uid.uidByte[3];
  } else {
    // if nothing is present  
    resp.b0 = 0;
    resp.b1 = 0;
    resp.b2 = 0;
    resp.b3 = 0;
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}


// ros service server for offering GetRFIDUID
ros::ServiceServer<arduino_msgs::rfiduid::Request, arduino_msgs::rfiduid::Response> server("GetRFIDUID", &get_rfid_uid);

// publisher for lever on dobot tool
std_msgs::Bool collision_msg;
ros::Publisher lever("DobotToolCollision", &collision_msg);


void setup() {
  // init ros node
  n.initNode();
  n.advertiseService(server);
  n.advertise(lever);
  

  // put your setup code here, to run once:
  SPI.begin();
  rfid.PCD_Init();
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  pinMode(8,INPUT_PULLUP);

}

void loop() {
  int collision_port_value = digitalRead(8);
  if(collision_port_value == LOW){
    collision_msg.data = 1;
  } else {
    collision_msg.data = 0;
  }
  lever.publish(&collision_msg);
  delay(16);
  n.spinOnce();
}
