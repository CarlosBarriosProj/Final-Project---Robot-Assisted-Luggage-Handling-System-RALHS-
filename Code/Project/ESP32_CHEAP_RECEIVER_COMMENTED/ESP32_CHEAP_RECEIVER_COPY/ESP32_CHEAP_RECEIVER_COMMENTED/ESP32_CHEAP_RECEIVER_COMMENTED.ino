// CARLOS ANTONIO BARRIOS GUZMAN || FINAL PROJECT || 19/04/2026 || MAIN BRAIN (MOTOR DRIVER, MPU, RFID MODULE, TRANSMITTER AND RECEIVER, LEDs, ETC)

#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <MQTT.h>
#include <math.h>
#include <BTS7960.h>
#include <SPI.h>
#include <MFRC522.h>



WiFiMulti WIFI_MULTI; // CREATE WiFiMulti OBJECT
MPU6050 mpu; // CREATE MPU OBJECT 
WiFiClient WIFI_CLIENT; // CREATE WiFiClient OBJECT
MQTTClient MQTT_CLIENT; // CREATE MQTTCLient OBJECT

String MQTT_TAG_ID = "";
String CARD_ID = "";



// ======= DECLARE FLOAT VARIABLES =======

float GYRO_Z = 0.0; 
float GYRO_OFF_SET = 0.0;
float CURRENT_HEADING = 0.0;
float TARGET_HEADING = 0.0;
const float WING_OFFSET_CM = 19.0f;          // DISTANCE FROM SIDE ULTRASONIC TO WING TIP 
const float WING_SAFE_MARGIN_CM = 10.0f;      // EXTRA SAFETY MARGIN
float HOME_FORWARD_HEADING = 0.0f;
float ALIGN_TARGET_HEADING = 0.0f;
const float HOME_REACHED_TOLERANCE = 0.20f; // TOLERANCE FOR DETECTING THAT THE CAR HAS REACHED THE HOME POSITION
volatile float ANCHOR_DISTANCE_1 = NAN;   // 0x0085
volatile float ANCHOR_DISTANCE_2 = NAN;   // 0x0084
const float MIN_DIST = 1.50;
const float MAX_DIST = 4.50;
float THRESHOLD = 0.17;

// ============================================================



// ======= DECLARE INT VARIABLES =======

const int RETURN_OBSTACLE_DISTANCE = 70;
int HOME_STABLE_COUNT = 0;
const int HOME_STABLE_REQUIRED = 5;
int FRONT_DISTANCE = 0;
volatile int NUMBER_TAPS = 0;
int LEFT_RETURN_DISTANCE = -1;
int RIGHT_RETURN_DISTANCE = -1;

// ============================================================



// ======= DECLARE BOOL VARIABLES / FUNCTIONS =======

bool LCD_ENABLED = true;
bool RETURN_STATE_ENTERED = false;
bool LCD_HOLD_ACTIVE = false;
bool HOME_HEADING_SAVED = false;
bool OBSTACLE_STOPPED = false;
bool ALIGN_TARGET_SET = false;
bool STRAIGHT_MODE = false;
volatile bool SENSOR_ACTIVE = false;
bool MQTT_STATUS = false;
bool RETURN_ACTION_TAKEN = false;
static bool enteredFinalHomeTurn = false; 
bool TURN_LEFT_RETURN_SAFE(unsigned long durationMs, int pwmValue);
bool TURN_RIGHT_RETURN_SAFE(unsigned long durationMs, int pwmValue);
bool MOVE_FORWARD_RETURN_SAFE(int pwmValue, unsigned long durationMs);

// ============================================================



// ======= DECLARE LONG VARIABLES =======

const unsigned long SAFE_STEP_MS = 20;   // check sensors every 20 ms
unsigned long RETURN_STATE_START = 0;
const unsigned long HOME_CHECK_DELAY = 1000; // ignore first 1 second of state 4
unsigned long LCD_HOLD_START = 0;
unsigned long LCD_HOLD_DURATION = 0;
unsigned long PREV_GYRO_MICROS = 0;
unsigned long DURATION = 0;
const unsigned long WAIT_TIME = 300000;
unsigned long PREVIOUS_TIME = 0;

// ============================================================



// ======= MAC ADDRESS OF RECEIVERS (ESP-NOW PROTOCOL)  =======

uint8_t BROADCAST_ADD_1[] = {0x34, 0x98, 0x7A, 0x73, 0x9A, 0x40};
uint8_t BROADCAST_ADD_2[] = {0x34, 0x98, 0x7A, 0x73, 0x7B, 0xD8};
uint8_t BROADCAST_ADD_3[] = {0x34, 0x98, 0x7A, 0x72, 0x24, 0x64};
uint8_t BROADCAST_ADD_4[] = {0x34, 0x98, 0x7A, 0x72, 0x1A, 0xD4};
uint8_t BROADCAST_ADD_5[] = {0xB8, 0xD6, 0x1A, 0x13, 0x36, 0x0C};

// ============================================================



// ======== SEND LCD DISPLAY ========

char LCD_LATER_LINE_1[17];
char LCD_LATER_LINE_2[17];

typedef struct __attribute__((packed)) MESSAGE_TEXT {

  char line1[17];  // fits 16x2 LCD
  char line2[17];

} MESSAGE_TEXT;

MESSAGE_TEXT lcdMessage;

void SEND_LCD_MESSAGE(const char* l1, const char* l2) {

  if (!LCD_ENABLED) return;

  strncpy(lcdMessage.line1, l1, sizeof(lcdMessage.line1));
  lcdMessage.line1[sizeof(lcdMessage.line1) - 1] = '\0';

  strncpy(lcdMessage.line2, l2, sizeof(lcdMessage.line2));
  lcdMessage.line2[sizeof(lcdMessage.line2) - 1] = '\0';

  esp_now_send(BROADCAST_ADD_5, (uint8_t*)&lcdMessage, sizeof(lcdMessage));}

// ============================================================



// ======= DEFINE I/O PINS =======

#define ESPNOW_CHANNEL 0
#define ECHO_PIN  34 // INPUT (LEVEL/SHIFTER)
#define TRIG_PIN 33 // OUTPUT(DIRECT);
#define EMERGENCY_BUZZER 5
#define SENSOR_INDICATOR 17 //TX2

// ============================================================



//  ======= RFID PIN SETUP =======

#define SS_PIN 32
#define RESET_PIN 27
#define SCK_PIN 18 
#define MISO_PIN 12
#define MOSI_PIN 23 

MFRC522 rfid(SS_PIN, RESET_PIN);

// ============================================================



// ======= BTS7960 SETUP ======= 

#define RIGHT_PWM_1 4 
#define LEFT_PWM_1 19 
#define L_EN_1 15
#define R_EN_1 2 

#define RIGHT_PWM_2 13 
#define LEFT_PWM_2 14 
#define L_EN_2 25
#define R_EN_2 26 

BTS7960 MOTOR_1(L_EN_1, R_EN_1, LEFT_PWM_1, RIGHT_PWM_1);
BTS7960 MOTOR_2(L_EN_2, R_EN_2, LEFT_PWM_2, RIGHT_PWM_2);

// ============================================================



// ======= WIFI SECTION =======

const char* SSID_1 = "iPhone";
const char* PASSWORD_1 = "204618J1";
const char* MQTT_NAME = "Mqtt Docker";
const char* BROKER_MQTT = "172.20.10.4";

//const char* SSID_1 = "Calo";
//const char* PASSWORD_1 = "movistarplus";
//const char* MQTT_NAME = "CARLOS_BARRIOS_MQTT";
//const char* BROKER_MQTT = "broker.hivemq.com";

// ============================================================



// ======= ESP-NOW SEND (NUMBER_TAPS) =======
typedef struct __attribute__((packed)) STRUCT_TAPS_SEND {

  volatile int TAPS;

} STRUCT_TAPS_SEND;

STRUCT_TAPS_SEND TapsData;
// ============================================================



// ======= RECEIVE DISTANCES =======
typedef struct __attribute__((packed)) MESSAGE_RECEIVED{
  float DIST_84;
  float DIST_85;
} MESSAGE_RECEIVED;

MESSAGE_RECEIVED myData;
// ============================================================


// ======= RECEIVE DISTANCE FROM LEFT AND RIGHT ULTRASONIC SENSORS =======
typedef struct __attribute__((packed)) ULTRA_DISTANCE_RECEIVED {
  int ULTRA_DISTANCE_1;
  int ULTRA_DISTANCE_2;
} ULTRA_DISTANCE_RECEIVED;

ULTRA_DISTANCE_RECEIVED ultraData;
// ============================================================


// ======= FUNCTION TO RECEIVE ESP-NOW DATA PACKETS FROM OTHER ESP32 MODULES =======
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  if (len == sizeof(MESSAGE_RECEIVED)) {

    memcpy(&myData, data, sizeof(myData));

    ANCHOR_DISTANCE_1 = myData.DIST_85;
    ANCHOR_DISTANCE_2 = myData.DIST_84;

    Serial.println("Received ANCHOR distances");
  }
  else if (len == sizeof(ULTRA_DISTANCE_RECEIVED)) {

    memcpy(&ultraData, data, sizeof(ultraData));

    LEFT_RETURN_DISTANCE = ultraData.ULTRA_DISTANCE_1;
    RIGHT_RETURN_DISTANCE = ultraData.ULTRA_DISTANCE_2;


    Serial.print("LEFT = ");
    Serial.print(LEFT_RETURN_DISTANCE);
    Serial.print(" || RIGHT = ");
    Serial.println(RIGHT_RETURN_DISTANCE);
  }
  else {
    Serial.printf("Unknown packet size: %d\n", len);
  }
}
// ============================================================

void setup() {

// IN THIS SETUP FUNCTION I INITIALIZE ALL THE MAIN HARDWARE AND COMMUNICATION MODULES BEFORE THE CAR STARTS WORKING.
// HERE I START THE MOTORS, RFID, MPU6050, WIFI, MQTT, AND ESP-NOW SO THE WHOLE SYSTEM IS READY BEFORE ENTERING THE MAIN LOOP.

// ======= BEGIN & ENABLE && FUNCTIONS =====
  Serial.begin(115200);
  delay(500);
  
  MOTOR_1.begin();
  MOTOR_1.enable();

  MOTOR_2.begin();
  MOTOR_2.enable();

  MQTT_CLIENT.begin(BROKER_MQTT, WIFI_CLIENT);
  MQTT_CLIENT.onMessage(MQTT_MESSAGE); 

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();

  WIFI_MULTI.addAP(SSID_1, PASSWORD_1);
  WiFi.mode(WIFI_STA);

  INIT_MPU(); // INITIALIZE MPU (FUNCTION)
  CALIBRATE_GYRO(); // CALIBRATE GYROSCOPE (FUNCTION)
  ACTIVE_DEVICE(); // FUNCTION TO CHECK IF CONNECTED TO WIFI AND MQTT SERVER (SUBSCRIBE TO TOPIC)    
// ============================================================



  HOME_FORWARD_HEADING = CURRENT_HEADING;
  HOME_HEADING_SAVED = true;

  Serial.print("HOME_FORWARD_HEADING = ");
  Serial.println(HOME_FORWARD_HEADING);



// ======= PIN MODE AS I/O =======

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SENSOR_INDICATOR, OUTPUT);
  pinMode(EMERGENCY_BUZZER, OUTPUT);

// ============================================================


// ======= PRINT TO SERIAL MONITOR (DEBUG) =======
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Sender WiFi channel: ");
  Serial.println(WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;}
// ============================================================
    


// ======= ADD PEERS (RECEIVERS) TO SEND DATA =======


    esp_now_peer_info_t peerInfo = {};


// ======= CHECK IF PEER 1 WAS SUCCESSFULLY ADDED ======= 
    memcpy(peerInfo.peer_addr, BROADCAST_ADD_1, 6);
    peerInfo.channel = ESPNOW_CHANNEL;   // must match receiver channel
    peerInfo.encrypt = false;
    
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer 1");
    } else {
      Serial.println("ESP-NOW peer 1 added successfully");}
// ============================================================


    memset(&peerInfo, 0, sizeof(peerInfo)); // Reset peerInfo to allow more data (BROADCAST_ADD_2);

 
// ======= CHECK IF PEER 2 WAS SUCCESSFULLY ADDED =======     
    memcpy(peerInfo.peer_addr, BROADCAST_ADD_2, 6);
    peerInfo.channel = ESPNOW_CHANNEL;   // must match receiver channel
    peerInfo.encrypt = false;
    
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer 2");
    } else {
      Serial.println("ESP-NOW peer 2 added successfully");}
// ============================================================


    memset(&peerInfo, 0, sizeof(peerInfo)); // Reset peerInfo to allow more data (BROADCAST_ADD_3);


// ======= CHECK IF PEER 3 WAS SUCCESSFULLY ADDED =======    
    memcpy(peerInfo.peer_addr, BROADCAST_ADD_3, 6);
    peerInfo.channel = ESPNOW_CHANNEL;   // must match receiver channel
    peerInfo.encrypt = false;
    
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer 3");
    } else {
      Serial.println("ESP-NOW peer 3 added successfully");}
// ============================================================


    memset(&peerInfo, 0, sizeof(peerInfo)); // Reset peerInfo to allow more data (BROADCAST_ADD_4);

    
// ======= CHECK IF PEER 4 WAS SUCCESSFULLY ADDED =======        
    memcpy(peerInfo.peer_addr, BROADCAST_ADD_4, 6);
    peerInfo.channel = ESPNOW_CHANNEL;   // must match receiver channel
    peerInfo.encrypt = false;
    
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer 4");
    } else {
      Serial.println("ESP-NOW peer 4 added successfully");}
// ============================================================


     memset(&peerInfo, 0, sizeof(peerInfo)); // Reset peerInfo to allow more data (BROADCAST_ADD_5);


// ======= CHECK IF PEER 5 WAS SUCCESSFULLY ADDED =======      
    memcpy(peerInfo.peer_addr, BROADCAST_ADD_5, 6);
    peerInfo.channel = ESPNOW_CHANNEL;   // must match receiver channel
    peerInfo.encrypt = false;
    
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer 5");
    } else {
      Serial.println("ESP-NOW peer 5 added successfully");}
// ============================================================


  esp_now_register_recv_cb(onRecv); 
  Serial.println("Receiver ready");}


// ============================================================

void loop() {

// THIS LOOP WORKS AS THE MAIN STATE MACHINE OF THE CAR.
// I USE NUMBER_TAPS AS THE MAIN CONTROL VARIABLE TO DECIDE WHAT THE CAR SHOULD DO:
// 0 = WAITING FOR A VALID USER, 1 OR 2 = FOLLOW USER MODE, 3 = TURN TO FACE HOME,
// 4 = RETURN HOME, AND 5 = FINAL ALIGNMENT WHEN THE CAR REACHES HOME.

  int tapsSnapshot = NUMBER_TAPS;

  MQTT_CLIENT.loop(); //
  UPDATE_LCD_HOLD(); //
  delay(10);


  if (!MQTT_CLIENT.connected()) {

    if (tapsSnapshot == 0) {
      ACTIVE_DEVICE();
    }
  }

  REBOOT_MQTT(); // FUNCTION TO SEND DATA EVERY 500ms TO KEEP MQTT CONNECTION ALIVE OVER TIME.
  UPDATE_HEADING(); // FUNCTION TO UPDATE THE CURRENT HEADING OF THE CAR USING THE GYROSCOPE Z-AXIS DATA 
  MQTT_ACTIVE_USER(); // FUNCTION TO LOG OUT A USER IF THEY DO NOT LOG IN WITHIN 30 SECONDS
  COUNTER_TAPS(); // // FUNCTION THAT CONSTANTLY CHECKS AND SENDS NUMBER_TAPS THROUGH ESP-NOW

  RFID_ENABLE(); // FUNCTION TO GRANT OR DENY ACCESS TO AUTHORIZED OR UNAUTHORIZED USERS

  GET_DATA(); // PRINT DATA TO SERIAL MONITOR (DEBUG).

  if ((tapsSnapshot == 1) || (tapsSnapshot == 2)) { // IF NUMBER_TAPS == 1 OR 2 (FOLLOW USER TAG)

    REBOOT_MQTT();
    UPDATE_FRONT_DISTANCE(); // FUNCTION TO CONSTANTLY CHECK THE DISTANCE FROM THE FRONT ULTRASONIC SENSOR
    SENSOR_INDICATOR_LED(); // FUNCTION TO INDICATE THE STATUS OF THE VEHICLE (OCCUPIED, AVAILABLE, OR RETURNING HOME)
    E_STOP(); // EMERGENCY STOP (BASED ON UPDATE_FRONT_DISTANCE)
    FOLLOW_TAG(); // FUNCTION TO FOLLOW THE USER TAG ONLY
  }

  if (tapsSnapshot == 3) { // IF NUMBER_TAPS == 3 (CAR TURNS TO FACE THE HOME POSITION)

    REBOOT_MQTT();

    if (TURN_FACE_HOME()) {

      ANCHOR_DISTANCE_1 = NAN; // SET ANCHOR_DISTANCE_1 TO "NOT A NUMBER" TO AVOID USING OLD DATA
      ANCHOR_DISTANCE_2= NAN; // SET ANCHOR_DISTANCE_2 TO "NOT A NUMBER" TO AVOID USING OLD DATA
      delay(500);
      NUMBER_TAPS = 4;

      RETURN_STATE_ENTERED = false;
      RETURN_STATE_START = 0;
      HOME_STABLE_COUNT = 0;

      Serial.println("STATE 3 complete -> entering STATE 4");
    }

    return;
  }

  if (tapsSnapshot == 4) { // IF NUMBER_TAPS == 4 (FOLLOW HOME_TAG)

    REBOOT_MQTT();
    UPDATE_FRONT_DISTANCE();

    if (!RETURN_STATE_ENTERED) {
      RETURN_STATE_ENTERED = true;
      RETURN_STATE_START = millis();
      HOME_STABLE_COUNT = 0;

      Serial.println("Entered STATE 4 (RETURN HOME)");
    }


    if (RETURN_OBSTACLE_HANDLER()) { 
      return;
    }

    bool obstacleDetected = (FRONT_DISTANCE > 0 && FRONT_DISTANCE <= RETURN_OBSTACLE_DISTANCE);

    if (!obstacleDetected) {
      if (RETURN_TAG_CLOSE_ENOUGH()) { // FUNCTION TO CHECK IF THE CAR IS CLOSE ENOUGH TO THE HOME TAG
        Serial.println("RETURN MODE: CLOSE -> FOLLOW_RETURN()");
        FOLLOW_RETURN(); // IF HOME_TAG IS WITHIN 5 METERS, CAR WILL FOLLOW THE TAG, FOLLOW USING THIS FUNCTION
      }
      else {
        Serial.println("RETURN MODE: FAR -> FOLLOW_RETURN_FAR()");
        FOLLOW_RETURN_FAR(); // IF HOME_TAG IS GREATER THAN 5 METERS, FOLLOW USING THIS FUNCTION
      }
    }

    Serial.print("STATE4 CHECK -> D1 = ");
    Serial.print(ANCHOR_DISTANCE_1);
    Serial.print(" || D2 = ");
    Serial.print(ANCHOR_DISTANCE_2);
    Serial.print(" || TOL = ");
    Serial.print(HOME_REACHED_TOLERANCE);
    Serial.print(" || ELAPSED = ");
    Serial.println(millis() - RETURN_STATE_START);

    bool validHomeRead =
        !isnan(ANCHOR_DISTANCE_1) && !isnan(ANCHOR_DISTANCE_2) &&
        ANCHOR_DISTANCE_1 <= HOME_REACHED_TOLERANCE &&
        ANCHOR_DISTANCE_2 <= HOME_REACHED_TOLERANCE;

    if (millis() - RETURN_STATE_START >= HOME_CHECK_DELAY) {

      if (validHomeRead) {
        HOME_STABLE_COUNT++;

        Serial.print("HOME candidate detected -> stable count = ");
        Serial.print(HOME_STABLE_COUNT);
        Serial.print(" || D1 = ");
        Serial.print(ANCHOR_DISTANCE_1);
        Serial.print(" || D2 = ");
        Serial.println(ANCHOR_DISTANCE_2);

        if (HOME_STABLE_COUNT >= HOME_STABLE_REQUIRED) { // IF THE CAR IS WITHIN TOLERANCE OF THE HOME TAG

          MOTOR_1.stop(); // STOP THE CAR
          MOTOR_2.stop(); // STOP THE CAR

          Serial.println("HOME REACHED - SWITCHING TO FINAL TURN");

          NUMBER_TAPS = 5;

          ANCHOR_DISTANCE_1 = NAN;
          ANCHOR_DISTANCE_2 = NAN;

          STRAIGHT_MODE = false;
          ALIGN_TARGET_SET = false;

          RETURN_STATE_ENTERED = false;
          RETURN_STATE_START = 0;
          HOME_STABLE_COUNT = 0;
        }
      }
      else {
        if (HOME_STABLE_COUNT != 0) {
          Serial.println("HOME candidate lost -> counter reset");
        }
        HOME_STABLE_COUNT = 0;
      }
    }

    return;
  }

  if (tapsSnapshot == 5){ //IF NUMBER_TAPS == 5(DO A FINAL ALIGNMENT TO FACE THE DIRECTION THE CAR CAME FROM)

    REBOOT_MQTT();

    if(!enteredFinalHomeTurn){
      enteredFinalHomeTurn = true;

      Serial.print("- Final Alignment at HOME");
      STRAIGHT_MODE = false;
      ALIGN_TARGET_SET = false;

    }

    if (!ALIGN_TARGET_SET) {
      ALIGN_TARGET_HEADING = HOME_FORWARD_HEADING;
      ALIGN_TARGET_SET = true;

      Serial.print("ALIGN_TARGET_HEADING (home forward) = ");
      Serial.println(ALIGN_TARGET_HEADING);
    }

    if (ALIGN_TO_TARGET(ALIGN_TARGET_HEADING)) {
      Serial.println("Final home heading reached");
      enteredFinalHomeTurn = false;
      RESET_NEW_CYCLE();
    }

    return;
  }
  else {
    enteredFinalHomeTurn = false;
  }
}

void E_STOP() {

// THIS FUNCTION IS MY MAIN SAFETY STOP DURING USER-FOLLOWING MODE.
// IF THE USER IS WITHIN VALID RANGE AND AN OBSTACLE IS DETECTED IN FRONT OF THE CAR,
// THE MOTORS STOP IMMEDIATELY AND THE BUZZER TURNS ON TO INDICATE AN EMERGENCY STOP.

  if (!USER_IN_VALID_RANGE()) {
    OBSTACLE_STOPPED = false;
    digitalWrite(EMERGENCY_BUZZER, LOW);
    return;
  }

  if (FRONT_DISTANCE > 0 && FRONT_DISTANCE <= 40) {
    OBSTACLE_STOPPED = true;
    digitalWrite(EMERGENCY_BUZZER, HIGH);
    MOTOR_1.stop();
    MOTOR_2.stop();
  } 
  else {
    OBSTACLE_STOPPED = false;
    digitalWrite(EMERGENCY_BUZZER, LOW);
  }
}

void UPDATE_FRONT_DISTANCE() {

// IN THIS FUNCTION I READ THE FRONT ULTRASONIC SENSOR.
// I SEND THE TRIGGER PULSE, MEASURE THE ECHO TIME, AND CONVERT THAT TIME INTO DISTANCE IN CENTIMETERS.
// IF NO ECHO IS RECEIVED BEFORE THE TIMEOUT, I MARK THE SENSOR AS INACTIVE.

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  DURATION = pulseIn(ECHO_PIN, HIGH, 30000); // TIMEOUT

  if (DURATION == 0) {

    FRONT_DISTANCE = -1; 
    SENSOR_ACTIVE = false;

  } else {

    FRONT_DISTANCE = DURATION * 0.034 / 2;
    SENSOR_ACTIVE = true;
  }
}

void ACTIVE_DEVICE(){

// THIS FUNCTION CONNECTS THE SYSTEM TO WIFI FIRST AND THEN TO THE MQTT SERVER.
// IT KEEPS TRYING UNTIL BOTH CONNECTIONS ARE SUCCESSFUL SO THE CAR CAN RECEIVE THE USER ID AND EXCHANGE DATA PROPERLY.

  Serial.print("Connecting Wifi. . .");
  while (WIFI_MULTI.run() != WL_CONNECTED){

    Serial.print(" .");
    delay(1000);
  }

  Serial.println("  Successfully Connected to Wifi");
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Receiver WiFi channel: ");
  Serial.println(WiFi.channel());

  Serial.print("Connecting to MQTT. . .");
  while(!MQTT_CLIENT.connect(MQTT_NAME)){

    Serial.print(" ."); 
    delay(1000);
  }

  Serial.println();

  Serial.println("Successfully Connected to MQTT Server");
  MQTT_CLIENT.subscribe("airport/robot/call");
  //MQTT_CLIENT.subscribe("RFID/MQTT/PROTOCOL/ARAJACINTO");
}

void MQTT_MESSAGE(String TOPIC, String IN_MESSAGE){ // FUNCTION TO RECEIVE THE USER ID FROM THE MQTT SERVER FOR ACCESS VALIDATION

  // HERE I RECEIVE THE AUTHORIZED USER ID FROM THE MQTT TOPIC.
  // THIS ID IS STORED AND LATER COMPARED AGAINST THE RFID CARD ID SO THE SYSTEM ONLY ALLOWS AUTHORIZED USERS.

  if (MQTT_STATUS == false){
    Serial.println();
    Serial.print("Topic: ");
    Serial.println(TOPIC);
    Serial.print("Incoming Data: ");
    Serial.println(IN_MESSAGE);
    
    MQTT_TAG_ID = IN_MESSAGE;
    MQTT_TAG_ID.toUpperCase();
 
    PREVIOUS_TIME = millis();
    MQTT_STATUS = true;
  }

  else {
    Serial.print("A VALID ID HAS ALREADY BEEN REGISTERED"); // IF A VALID ID IS ALREADY REGISTERED, THE CODE WILL NOT ALLOW A NEW ONE.
  }
}

void MQTT_ACTIVE_USER (){

// THIS FUNCTION HANDLES THE MQTT USER TIMEOUT.
// IF AN ID WAS RECEIVED BUT THE USER DOES NOT START THE SERVICE IN TIME, I CLEAR THE STORED ID
// SO THE SYSTEM CAN GO BACK TO WAITING FOR A NEW VALID USER.

  if (MQTT_STATUS && NUMBER_TAPS == 0){

    unsigned long CURRENT_TIME = millis(); 

    if (CURRENT_TIME - PREVIOUS_TIME >= WAIT_TIME) {

      MQTT_STATUS = false;
      PREVIOUS_TIME = 0;
      MQTT_TAG_ID = "";
      Serial.println("MQTT ID timeout expired, ready for new ID");
    }
  }
}

void RFID_ENABLE() {

// THIS FUNCTION READS THE RFID CARD AND COMPARES IT WITH THE USER ID RECEIVED FROM MQTT.
// IT IS ALSO RESPONSIBLE FOR ADVANCING THE MAIN STATES OF THE SYSTEM USING NUMBER_TAPS:
// FIRST TAP STARTS THE SERVICE, SECOND TAP ASKS FOR CONFIRMATION, AND THIRD TAP ENDS THE RIDE.

  if (!rfid.PICC_IsNewCardPresent()){
    return;
  }

  if (!rfid.PICC_ReadCardSerial()){
    return;
  }

  CARD_ID = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
  
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(rfid.uid.uidByte[i], HEX);

    CARD_ID.concat(String(rfid.uid.uidByte[i] < 0x10 ? " 0" : " "));
    CARD_ID.concat(String(rfid.uid.uidByte[i], HEX));
  }

  Serial.println();

  CARD_ID.toUpperCase();

  if ((CARD_ID.substring(1) == (MQTT_TAG_ID)) && NUMBER_TAPS == 0) { // FIRST. WELCOME

    Serial.println(" --- Access Granted. Welcome --- ");
    SHOW_LCD_FOR(" Access Granted ", "--- Welcome. ---",
                 "  Following ID  ", "                ", 
                 2000);

   MQTT_STATUS = true;
   NUMBER_TAPS += 1;
   MQTT_CLIENT.publish("airport/robot/response", "DONE");
   //MQTT_CLIENT.publish("RFID/END/MESSAGE", "DONE");

  }

  else if ((CARD_ID.substring(1) == (MQTT_TAG_ID)) && NUMBER_TAPS == 1) { // SECOND. CONFIRMATION 
        
    Serial.println();
    Serial.println(" Are You Sure You Want to End Your Current Service?");
    SHOW_LCD_FOR("- Tap Again To -", "End Current Ride",
                 "  Following ID  ", "                ",
                  4000);

    NUMBER_TAPS += 1;

  }
      
  else if ((CARD_ID.substring(1) == MQTT_TAG_ID) && NUMBER_TAPS == 2) { //THIRD. END OF RIDE
      
    Serial.println("Thank You For Trusting Our Services. Have a Great Day");
    SHOW_LCD_FOR("-- Thank You --", "Have a Great Day",
                 "-- Finalizing --", "Ride.....",
                 3000);

    NUMBER_TAPS += 1; 
    PREVIOUS_TIME = 0;

    Serial.print(" FINALIZING RIDE.");

  }

  else if ((CARD_ID.substring(1) != MQTT_TAG_ID) && NUMBER_TAPS > 0) { //FOURTH. ALREADY AN EXISTING USER
    
    Serial.println(" --- ! Access Denied. Following an Existing User ! --- ");
    SHOW_LCD_FOR(" Access Denied! ", "Ride in Progress",
                 "  Following ID  ", "                ",
                 3000);
    
  }
    
  else {
    Serial.println(" --- ! Access Denied. Unauthorized ID !"); // UNAUTHORIZED ID
    SHOW_LCD_FOR(" Access Denied! ", "Unauthorized ID",
                 "- Waiting For - ", "An Authorized ID",
                 2000);
    
  }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();  

}

void GET_DATA(){

 static unsigned long t = 0;

  if (millis() - t > 500){

    t = millis();


Serial.printf("\n D1=%.2f || D2=%.2f || NUMBER_TAPS= %d || DISTANCE = %d\n",
                  ANCHOR_DISTANCE_1, ANCHOR_DISTANCE_2, NUMBER_TAPS, FRONT_DISTANCE);
 }
}

void COUNTER_TAPS() {

// THIS FUNCTION SENDS THE CURRENT NUMBER_TAPS VALUE TO THE OTHER ESP32 MODULES THROUGH ESP-NOW.
// I SEND IT AGAIN WHEN THE VALUE CHANGES OR AFTER A FIXED TIME SO ALL THE DEVICES STAY SYNCHRONIZED.
  
  static int lastTaps = -1;
  static unsigned long lastSendTime = 0;
  const unsigned long RESEND_INTERVAL = 1000;

  if (NUMBER_TAPS != lastTaps || millis() - lastSendTime >= RESEND_INTERVAL) {

    lastTaps = NUMBER_TAPS;
    lastSendTime = millis();
    TapsData.TAPS = NUMBER_TAPS;

    esp_err_t RESULT_1 = esp_now_send(BROADCAST_ADD_1, (uint8_t*)&TapsData, sizeof(TapsData));
    esp_err_t RESULT_2 = esp_now_send(BROADCAST_ADD_2, (uint8_t*)&TapsData, sizeof(TapsData));
    esp_err_t RESULT_3 = esp_now_send(BROADCAST_ADD_3, (uint8_t*)&TapsData, sizeof(TapsData));
    esp_err_t RESULT_4 = esp_now_send(BROADCAST_ADD_4, (uint8_t*)&TapsData, sizeof(TapsData));
    esp_err_t RESULT_5 = esp_now_send(BROADCAST_ADD_5, (uint8_t*)&TapsData, sizeof(TapsData));

    if (RESULT_1 == ESP_OK && RESULT_2 == ESP_OK && RESULT_3 == ESP_OK && RESULT_4 == ESP_OK && RESULT_5 == ESP_OK) {
      Serial.printf("Sent -> NUMBER OF TAPS: %d\n", TapsData.TAPS);
    } else {
      Serial.print("RESULT_1: "); Serial.println((int)RESULT_1);
      Serial.print("RESULT_2: "); Serial.println((int)RESULT_2);
      Serial.print("RESULT_3: "); Serial.println((int)RESULT_3);
      Serial.print("RESULT_4: "); Serial.println((int)RESULT_4);
      Serial.print("RESULT_5: "); Serial.println((int)RESULT_5);
    }
  }
}

void INIT_MPU(){
  
  Wire.begin(21, 22); // SDA, SCL
  mpu.initialize();

  Serial.println("MPU Initialized");

  if (mpu.testConnection()){
    Serial.println("MPU Successfully Connected");}
    
  else {Serial.println("MPU Failed to Connect");
        return;}

}

void CALIBRATE_GYRO(){

// BEFORE USING THE GYROSCOPE TO TRACK HEADING, I CALIBRATE ITS Z-AXIS OFFSET.
// I TAKE MANY SAMPLES WHILE THE CAR IS STILL, AVERAGE THEM, AND USE THAT VALUE AS THE OFFSET
// SO THE HEADING CALCULATION IS MORE STABLE AND LESS AFFECTED BY SENSOR BIAS.

  Serial.println("Calibrating Gyro . . . .");

  long SUM = 0;
  const int SAMPLES = 500;

  for (int i = 0; i < SAMPLES; i++){

   int16_t X_AXIS, Y_AXIS, Z_AXIS;
   mpu.getRotation(&X_AXIS, &Y_AXIS, &Z_AXIS);
   SUM += Z_AXIS;
   delay(5);}

   GYRO_OFF_SET = (float)SUM / SAMPLES;
   Serial.print("GYRO_OFF_SET = ");
   Serial.println(GYRO_OFF_SET);

   PREV_GYRO_MICROS = micros();
   
}

void UPDATE_HEADING(){

// IN THIS FUNCTION I UPDATE THE CURRENT HEADING OF THE CAR USING THE GYROSCOPE Z-AXIS.
// FIRST I READ THE ANGULAR VELOCITY, THEN I CALCULATE THE ELAPSED TIME dt,
// AND FINALLY I INTEGRATE THAT ROTATION OVER TIME TO ESTIMATE THE NEW HEADING.

  int16_t X_AXIS, Y_AXIS, Z_AXIS;
  mpu.getRotation(&X_AXIS, &Y_AXIS, &Z_AXIS);

  unsigned long NOW = micros();
  float dt = (NOW - PREV_GYRO_MICROS) / 1000000.0f;
  PREV_GYRO_MICROS = NOW;

  GYRO_Z = ((float)Z_AXIS - GYRO_OFF_SET) / 131.0f;

  CURRENT_HEADING += GYRO_Z * dt;

  CURRENT_HEADING = NORMALIZE_ANGLE(CURRENT_HEADING); // JUST ADDED
  
}

void FORWARD_STABLE() { // FUNCTION TO MAKE THE CAR MOVE AS STRAIGHT AS POSSIBLE

  // HERE I USE THE DIFFERENCE BETWEEN TARGET_HEADING AND CURRENT_HEADING TO CORRECT THE MOTORS.
  // IF THE CAR STARTS DRIFTING TO ONE SIDE, I CHANGE THE LEFT AND RIGHT PWM VALUES SO IT CAN RECOVER
  // AND KEEP MOVING FORWARD IN A STRAIGHTER PATH.

  const int BASE_PWM = 70;
  const float HEADING_DEADBAND = 3.0f;   // ignore tiny errors
  const float KP = 2.0f;                 // softer correction
  const int MAX_CORRECTION = 20;         // prevent strong arc correction

  float headingError = NORMALIZE_ANGLE(TARGET_HEADING - CURRENT_HEADING);

  if (fabs(headingError) < HEADING_DEADBAND) {
    headingError = 0.0f;
  }

  int correction = (int)(headingError * KP);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int pwmLeft = constrain(BASE_PWM + correction, 60, 120);
  int pwmRight = constrain(BASE_PWM - correction, 60, 120);

  Serial.print("TARGET_HEADING = ");
  Serial.print(TARGET_HEADING);
  Serial.print(" || CURRENT_HEADING = ");
  Serial.print(CURRENT_HEADING);
  Serial.print(" || HEADING_ERROR = ");
  Serial.print(headingError);
  Serial.print(" || PWM_LEFT = ");
  Serial.print(pwmLeft);
  Serial.print(" || PWM_RIGHT = ");
  Serial.println(pwmRight);

  MOTOR_1.pwm = pwmLeft;
  MOTOR_1.front();

  MOTOR_2.pwm = pwmRight;
  MOTOR_2.front();
}

float NORMALIZE_ANGLE(float ANGLE){ // FUNCTION TO KEEP THE ANGLE BETWEEN -180 AND 180 DEGREES

  while (ANGLE > 180.0f) ANGLE -= 360.0f;
  while(ANGLE < -180.0f) ANGLE += 360.0f;
  return ANGLE;

}

bool ALIGN_TO_TARGET(float targetHeading) { // FUNCTION TO ALIGN THE CAR WITH A TARGET HEADING

  // THIS FUNCTION IS USED WHEN I NEED THE CAR TO ROTATE UNTIL IT FACES A SPECIFIC ANGLE.
  // I CALCULATE THE ANGLE ERROR, REDUCE THE TURNING SPEED WHEN THE CAR GETS CLOSER TO THE TARGET,
  // AND ONLY ACCEPT THE ALIGNMENT AFTER SEVERAL STABLE READINGS TO AVOID FALSE STOPS.

  static int stableCount = 0;

  const float STOP_TOLERANCE = 20.0f;        // final acceptable error
  const float SLOW_ZONE = 50.0f;            // slow down here
  const int MIN_TURN_PWM = 70;              // minimum turning force
  const int MAX_TURN_PWM = 120;             // max turning force
  const int STABLE_REQUIRED = 4;            // require several stable loops

  float error = NORMALIZE_ANGLE(targetHeading - CURRENT_HEADING);
  float absError = fabs(error);

  Serial.print("CURRENT_HEADING = ");
  Serial.print(CURRENT_HEADING);
  Serial.print(" || TARGET = ");
  Serial.print(targetHeading);
  Serial.print(" || ERROR = ");
  Serial.println(error);

  if (absError <= STOP_TOLERANCE) { // IF ALIGNED, STOP THE CAR
    MOTOR_1.stop();
    MOTOR_2.stop();

    stableCount++;
    if (stableCount >= STABLE_REQUIRED) {
      stableCount = 0;
      return true;
    }
    return false;
  }

  stableCount = 0;

  // Proportional turn strength with slowdown near target
  float scale = absError / SLOW_ZONE;
  scale = constrain(scale, 0.0f, 1.0f);

  int turnPwm = MIN_TURN_PWM + (int)((MAX_TURN_PWM - MIN_TURN_PWM) * scale);
  turnPwm = constrain(turnPwm, MIN_TURN_PWM, MAX_TURN_PWM);

  // DIRECTION BASED ON LIVE ERRORS EVERY CYCLE
  if (error > 0.0f) {
    // TURN LEFT
    MOTOR_1.pwm = turnPwm;
    MOTOR_1.back();
    MOTOR_2.pwm = turnPwm;
    MOTOR_2.front();
  } 
  else {
    // TURN RIGHT
    MOTOR_1.pwm = turnPwm;
    MOTOR_1.front();
    MOTOR_2.pwm = turnPwm;
    MOTOR_2.back();
  }

  return false;
}

void FOLLOW_TAG() { // FOLLOW USER TAG

  // THIS FUNCTION CONTROLS THE NORMAL USER-FOLLOWING MODE.
  // I COMPARE BOTH ANCHOR DISTANCES TO DECIDE WHETHER THE CAR SHOULD TURN LEFT, TURN RIGHT,
  // OR LOCK ITS CURRENT HEADING AND MOVE FORWARD.

  // E-STOP IF OBJECT IN THE MIDDLE
  if (OBSTACLE_STOPPED) {
    MOTOR_1.stop();
    MOTOR_2.stop();
    return;
  }

  // STOP IF ANCHOR DATA IS INVALID OR USER IS OUT OR WITHIN MINIMUM RANGE
  if (isnan(ANCHOR_DISTANCE_1) || isnan(ANCHOR_DISTANCE_2) ||
      ANCHOR_DISTANCE_1 <= MIN_DIST || ANCHOR_DISTANCE_1 >= MAX_DIST ||
      ANCHOR_DISTANCE_2 <= MIN_DIST || ANCHOR_DISTANCE_2 >= MAX_DIST) {

    MOTOR_1.stop();
    MOTOR_2.stop();
    STRAIGHT_MODE = false;
    return;
  }


  float DIFFERENCE = ANCHOR_DISTANCE_1 - ANCHOR_DISTANCE_2;

  if (DIFFERENCE > THRESHOLD) { // RIGHT TURN WHEN FOLLOWING USER
    STRAIGHT_MODE = false;

    MOTOR_1.pwm = 80;
    MOTOR_1.back();

    MOTOR_2.pwm = 80;
    MOTOR_2.front();
  }
  else if (DIFFERENCE < -THRESHOLD) { // LEFT TURN WHEN FOLLOWING USER 
    STRAIGHT_MODE = false;

    MOTOR_1.pwm = 80;
    MOTOR_1.front();

    MOTOR_2.pwm = 80;
    MOTOR_2.back();
  }
  else {

    if (!STRAIGHT_MODE) { // GO FORWARD WHEN FOLLOWING USER
      TARGET_HEADING = CURRENT_HEADING;
      STRAIGHT_MODE = true;
      Serial.print("Locked TARGET_HEADING = ");
      Serial.println(TARGET_HEADING);
    }

    FORWARD_STABLE();
  }
}

void FOLLOW_RETURN() { // FOLLOW HOME_TAG OR HOME POSITION

  // THIS FUNCTION IS USED WHEN THE HOME TAG IS CLOSE ENOUGH FOR MORE PRECISE TRACKING.
  // I USE THE DIFFERENCE BETWEEN BOTH ANCHOR DISTANCES TO APPLY A PROPORTIONAL CORRECTION
  // SO THE CAR CAN APPROACH THE HOME POSITION MORE ACCURATELY.

  if (OBSTACLE_STOPPED) {
    MOTOR_1.stop();
    MOTOR_2.stop();
    return;
  }

  if (isnan(ANCHOR_DISTANCE_1) || isnan(ANCHOR_DISTANCE_2)) { 
    MOTOR_1.stop();
    MOTOR_2.stop();
    STRAIGHT_MODE = false;
    Serial.println("RETURN: waiting for valid anchor data");
    return;
  }

  const int BASE_PWM = 75;
  const float KP_RETURN = 120.0f;

  float DIFFERENCE = ANCHOR_DISTANCE_1 - ANCHOR_DISTANCE_2;
  int correction = (int)(DIFFERENCE * KP_RETURN);
  correction = constrain(correction, -25, 25);

  int pwmLeft = constrain(BASE_PWM - correction, 60, 120);
  int pwmRight = constrain(BASE_PWM + correction, 60, 120);

  MOTOR_1.pwm = pwmLeft;
  MOTOR_1.front();

  MOTOR_2.pwm = pwmRight;
  MOTOR_2.front();

  Serial.print("RETURN TRACKING -> DIFF = ");
  Serial.print(DIFFERENCE);
  Serial.print(" || LEFT PWM = ");
  Serial.print(pwmLeft);
  Serial.print(" || RIGHT PWM = ");
  Serial.println(pwmRight);
}

void RESET_NEW_CYCLE() { // RESET FUNCTION - FRESH START FOR NEXT OPERATION

  // WHEN A FULL SERVICE CYCLE FINISHES, I RESET THE MOST IMPORTANT VARIABLES HERE.
  // THIS LETS THE CAR GO BACK TO ITS INITIAL WAITING STATE WITHOUT USING OLD USER DATA,
  // OLD SENSOR VALUES, OR OLD HEADING TARGETS.

  NUMBER_TAPS = 0;

  MQTT_STATUS = false;
  MQTT_TAG_ID = "";
  CARD_ID = "";
  PREVIOUS_TIME = 0;

  ANCHOR_DISTANCE_1 = NAN;
  ANCHOR_DISTANCE_2 = NAN;

  STRAIGHT_MODE = false;
  TARGET_HEADING = 0.0f;

  ALIGN_TARGET_SET = false;
  ALIGN_TARGET_HEADING = 0.0f;


  MOTOR_1.stop();
  MOTOR_2.stop();

  Serial.println("System reset for new cycle");
}

void REBOOT_MQTT(){

// THIS FUNCTION PERIODICALLY SENDS A SMALL MQTT MESSAGE SO I CAN KEEP MONITORING THE SYSTEM STATUS.
// IN THIS CASE I SEND THE CURRENT HEADING AND GYRO Z VALUE AS DEBUG DATA.

      static unsigned long PREV_TIME = 0;
  
   if (millis() - PREV_TIME > 500) {

    PREV_TIME = millis();
    char payload[50];
     snprintf(payload, sizeof(payload), "H:%.2f,G:%.2f", CURRENT_HEADING, GYRO_Z);

     MQTT_CLIENT.publish("airport/robot/test", payload);

    //MQTT_CLIENT.publish("ACTIVE/MQTT", "ACTIVE");}
}}

bool USER_IN_VALID_RANGE() { // FUNCTION TO CHECK IF THE USER IS WITHIN THE RECOMMENDED DISTANCE RANGE

  return !isnan(ANCHOR_DISTANCE_1) && !isnan(ANCHOR_DISTANCE_2) &&
         ANCHOR_DISTANCE_1 > MIN_DIST && ANCHOR_DISTANCE_1 < MAX_DIST &&
         ANCHOR_DISTANCE_2 > MIN_DIST && ANCHOR_DISTANCE_2 < MAX_DIST;
}

bool TIMER_1(bool condition) { // FUNCTION TO CHECK IF AN OBSTACLE HAS BEEN DETECTED CONTINUOUSLY FOR 3 SECONDS

  // I USE THIS TIMER TO MAKE SURE THE OBSTACLE IS REAL AND NOT JUST A QUICK NOISE OR FALSE READING.
  // ONLY IF THE CONDITION STAYS TRUE FOR 3 SECONDS DOES THE FUNCTION RETURN TRUE.

  static unsigned long TIMER_1_PREV_TIME = 0;
  static bool timing = false;

  if (condition) {

    if (!timing) {
      TIMER_1_PREV_TIME = millis();
      timing = true;
    }

    if (millis() - TIMER_1_PREV_TIME >= 3000) {
      return true;
    }
  }
  else {
    timing = false;
  }

  return false;
}

bool RETURN_TAG_CLOSE_ENOUGH() { // FUNCTION TO CHECK IF THE CAR IS CLOSE ENOUGH TO THE HOME TAG

  return !isnan(ANCHOR_DISTANCE_1) && !isnan(ANCHOR_DISTANCE_2) &&
         ANCHOR_DISTANCE_1 <= 5.0f && ANCHOR_DISTANCE_2 <= 5.0f;
}


bool RETURN_OBSTACLE_HANDLER() { // FUNCTION TO DETECT AND HANDLE OBSTACLES AT THE FRONT, LEFT, OR RIGHT WHILE RETURNING HOME

  // THIS IS ONE OF THE MAIN DECISION-MAKING FUNCTIONS DURING RETURN MODE.
  // IF AN OBSTACLE STAYS IN FRONT OF THE CAR FOR SEVERAL SECONDS, I STOP THE CAR,
  // CHECK THE SIDE CLEARANCES, CHOOSE THE SAFEST TURNING SIDE, MOVE PAST THE OBSTACLE,
  // AND THEN REALIGN THE CAR TO ITS ORIGINAL RETURN HEADING.

  bool obstacleDetected = (FRONT_DISTANCE > 0 && FRONT_DISTANCE <= RETURN_OBSTACLE_DISTANCE);

  if (obstacleDetected) {

    OBSTACLE_STOPPED = true;
    MOTOR_1.stop();
    MOTOR_2.stop();

    if (TIMER_1(obstacleDetected) && !RETURN_ACTION_TAKEN) {

      RETURN_ACTION_TAKEN = true;

      float leftClearance = WING_CLEARANCE_CM(LEFT_RETURN_DISTANCE);
      float rightClearance = WING_CLEARANCE_CM(RIGHT_RETURN_DISTANCE);

      Serial.print("RETURN DECISION -> LEFT RAW: ");
      Serial.print(LEFT_RETURN_DISTANCE);
      Serial.print(" || RIGHT RAW: ");
      Serial.print(RIGHT_RETURN_DISTANCE);
      Serial.print(" || LEFT CLR: ");
      Serial.print(leftClearance);
      Serial.print(" || RIGHT CLR: ");
      Serial.println(rightClearance);

      bool moveOk = false;

      if (LEFT_WING_DANGER() && !RIGHT_WING_DANGER()) {
        Serial.println("LEFT wing too close -> forcing RIGHT turn");
        moveOk = TURN_RIGHT_RETURN_SAFE(2500, 90);
      }
      else if (RIGHT_WING_DANGER() && !LEFT_WING_DANGER()) {
        Serial.println("RIGHT wing too close -> forcing LEFT turn");
        moveOk = TURN_LEFT_RETURN_SAFE(2500, 90);
      }
      else if (leftClearance > rightClearance) {
        Serial.println("Turning LEFT around obstacle");
        moveOk = TURN_LEFT_RETURN_SAFE(2500, 90);
      }
      else {
        Serial.println("Turning RIGHT around obstacle");
        moveOk = TURN_RIGHT_RETURN_SAFE(2500, 90);
      }

      if (!moveOk) {
        Serial.println("Abort obstacle bypass during turn");
        RETURN_ACTION_TAKEN = false;
        return true;
      }

      delay(150);

      Serial.println("Moving forward past obstacle");
      moveOk = MOVE_FORWARD_RETURN_SAFE(100, 900);

      if (!moveOk) {
        Serial.println("Abort obstacle bypass during forward move");
        RETURN_ACTION_TAKEN = false;
        return true;
      }

      delay(150);

      Serial.println("Re-aligning to return heading");
      REALIGN_TO_RETURN_HEADING();

      OBSTACLE_STOPPED = false;
      STRAIGHT_MODE = false;
    }

    return true;
  }

  OBSTACLE_STOPPED = false;
  RETURN_ACTION_TAKEN = false;
  return false;
}

void FOLLOW_RETURN_FAR() { // FUNCTION TO FOLLOW RETURN TAG WHEN DISTANCE OUTSIDE THE 5 METERS MARK

  // THIS FUNCTION IS USED WHEN THE HOME TAG IS STILL FAR AWAY.
  // INSTEAD OF USING CLOSE-RANGE CORRECTION, I LOCK THE CURRENT HEADING AND KEEP THE CAR MOVING FORWARD
  // UNTIL IT GETS CLOSER AND CAN SWITCH TO THE MORE PRECISE RETURN TRACKING MODE.

  if (OBSTACLE_STOPPED) {
    MOTOR_1.stop();
    MOTOR_2.stop();
    return;
  }

  if (!STRAIGHT_MODE) {
    TARGET_HEADING = CURRENT_HEADING;
    STRAIGHT_MODE = true;

    Serial.print("Locked FAR RETURN TARGET_HEADING = ");
    Serial.println(TARGET_HEADING);
  }

  FORWARD_STABLE();
}

void REALIGN_TO_RETURN_HEADING() { // FUNCTION TO REALIGN THE CAR TO ITS ORIGINAL RETURN HEADING AFTER AVOIDING AN OBSTACLE

  // AFTER AVOIDING AN OBSTACLE, THE CAR MAY NO LONGER BE FACING THE SAME DIRECTION IT HAD BEFORE.
  // HERE I MAKE THE CAR ROTATE AGAIN UNTIL IT MATCHES THE STORED RETURN HEADING,
  // SO IT CAN CONTINUE GOING HOME IN THE CORRECT DIRECTION.

  ALIGN_TARGET_SET = false;
  ALIGN_TARGET_HEADING = TARGET_HEADING;


  unsigned long startTime = millis();

  while (!ALIGN_TO_TARGET(ALIGN_TARGET_HEADING)) {
    UPDATE_HEADING();

    // safety timeout so it does not get stuck forever
    if (millis() - startTime > 3000) {
      Serial.println("REALIGN timeout");
      break;
    }
  }

  MOTOR_1.stop();
  MOTOR_2.stop();
  STRAIGHT_MODE = false;
}

void SHOW_LCD_FOR(const char* NOW_1, const char* NOW_2, 
                  const char* LATER_1, const char* LATER_2,
                  unsigned long TIME_HELD){ // FUNCTION TO DISPLAY INFORMATION ON THE LCD


    SEND_LCD_MESSAGE(NOW_1, NOW_2);

    strncpy(LCD_LATER_LINE_1, LATER_1, sizeof(LCD_LATER_LINE_1));
    LCD_LATER_LINE_1[sizeof(LCD_LATER_LINE_1) - 1] = '\0';

    strncpy(LCD_LATER_LINE_2, LATER_2, sizeof(LCD_LATER_LINE_2));
    LCD_LATER_LINE_2[sizeof(LCD_LATER_LINE_2) - 1] = '\0';
    
    LCD_HOLD_ACTIVE = true;
    LCD_HOLD_START = millis();
    LCD_HOLD_DURATION = TIME_HELD; }

void UPDATE_LCD_HOLD(){ // FUNCTION TO KEEP THE MESSAGE ON THE LCD LONG ENOUGH FOR THE USER TO READ IT

  if (LCD_HOLD_ACTIVE && millis() - LCD_HOLD_START >= LCD_HOLD_DURATION){

    SEND_LCD_MESSAGE(LCD_LATER_LINE_1, LCD_LATER_LINE_2);
    LCD_HOLD_ACTIVE = false;
  }


}

void SENSOR_INDICATOR_LED(){ // ULTRASONIC INDICATOR (DEBUG)

  if (SENSOR_ACTIVE){

    digitalWrite(SENSOR_INDICATOR, HIGH);}

  else {digitalWrite(SENSOR_INDICATOR, LOW);}

}

bool TURN_FACE_HOME() { // FUNCTION TO ROTATE THE CAR 180 DEGREES SO IT CAN FACE THE HOME POSITION BEFORE RETURNING

  // BEFORE STARTING THE RETURN, I USE THE HEADING SAVED AT THE BEGINNING OF THE RIDE.
  // BY ADDING 180 DEGREES, I MAKE THE CAR TURN AROUND AND FACE THE HOME DIRECTION.

  if (!HOME_HEADING_SAVED) return true;

  if (!ALIGN_TARGET_SET) {
    ALIGN_TARGET_HEADING = NORMALIZE_ANGLE(HOME_FORWARD_HEADING + 180.0f);
    ALIGN_TARGET_SET = true;

    
    Serial.print("TURN_FACE_HOME -> target: ");
    Serial.println(ALIGN_TARGET_HEADING);
  }

  UPDATE_HEADING();

  bool done = ALIGN_TO_TARGET(ALIGN_TARGET_HEADING);
  if (done) {
    ALIGN_TARGET_SET = false;
  }

  return done;
}

bool SIDE_READING_VALID(int d) { // FUNCTION TO CHECK IF A SIDE ULTRASONIC SENSOR READING IS VALID
  return d >= 0;
}

float WING_CLEARANCE_CM(int sideDistance) { // FUNCTION TO CALCULATE THE CLEARANCE BETWEEN THE SIDE OBSTACLE READING AND THE CAR WING
  if (!SIDE_READING_VALID(sideDistance)) return 999.0f; // treat invalid as far for now
  float clearance = (float)sideDistance - WING_OFFSET_CM;
  return clearance;
}

bool LEFT_WING_DANGER() { // FUNCTION TO CHECK IF THE LEFT SIDE OF THE CAR IS TOO CLOSE TO AN OBSTACLE
  if (!SIDE_READING_VALID(LEFT_RETURN_DISTANCE)) return false;
  return LEFT_RETURN_DISTANCE <= (WING_OFFSET_CM + WING_SAFE_MARGIN_CM);
}

bool RIGHT_WING_DANGER() { // FUNCTION TO CHECK IF THE RIGHT SIDE OF THE CAR IS TOO CLOSE TO AN OBSTACLE
  if (!SIDE_READING_VALID(RIGHT_RETURN_DISTANCE)) return false;
  return RIGHT_RETURN_DISTANCE <= (WING_OFFSET_CM + WING_SAFE_MARGIN_CM);
}

bool ANY_WING_DANGER() { // FUNCTION TO CHECK IF EITHER SIDE OF THE CAR IS TOO CLOSE TO AN OBSTACLE
  return LEFT_WING_DANGER() || RIGHT_WING_DANGER();
}

void STOP_RETURN_MOTION() { // FUNCTION TO IMMEDIATELY STOP THE CAR DURING RETURN OR OBSTACLE AVOIDANCE
  MOTOR_1.stop();
  MOTOR_2.stop();
  OBSTACLE_STOPPED = true;
}

bool TURN_LEFT_RETURN_SAFE(unsigned long durationMs, int pwmValue = 90) { // FUNCTION TO TURN LEFT SAFELY (CONSIDERING LEFT WING DISTANCE)

  // THIS FUNCTION MAKES THE CAR TURN LEFT FOR A CONTROLLED AMOUNT OF TIME WHILE STILL CHECKING SAFETY.
  // IF THE SIDE CLEARANCE BECOMES DANGEROUS DURING THE TURN, I STOP THE CAR IMMEDIATELY.
  unsigned long start = millis();

  while (millis() - start < durationMs) {
    MOTOR_1.pwm = pwmValue;
    MOTOR_1.back();

    MOTOR_2.pwm = pwmValue;
    MOTOR_2.front();

    // keep sensors fresh during motion if possible
    UPDATE_FRONT_DISTANCE();

    if (ANY_WING_DANGER()) {
      STOP_RETURN_MOTION();
      Serial.println("EMERGENCY STOP: wing danger during LEFT turn");
      return false;
    }

    delay(SAFE_STEP_MS);
  }

  MOTOR_1.stop();
  MOTOR_2.stop();
  return true;
}

bool TURN_RIGHT_RETURN_SAFE(unsigned long durationMs, int pwmValue = 90) { // FUNCTION TO TURN RIGHT SAFELY (CONSIDERING RIGHT WING DISTANCE)

  // THIS FUNCTION DOES THE SAME AS THE LEFT SAFE TURN BUT FOR THE RIGHT SIDE.
  // I KEEP CHECKING THE SENSORS DURING THE MOVEMENT SO THE CAR CAN STOP IF THE CLEARANCE BECOMES UNSAFE.
  unsigned long start = millis();

  while (millis() - start < durationMs) {
    MOTOR_1.pwm = pwmValue;
    MOTOR_1.front();

    MOTOR_2.pwm = pwmValue;
    MOTOR_2.back();

    UPDATE_FRONT_DISTANCE();

    if (ANY_WING_DANGER()) {
      STOP_RETURN_MOTION();
      Serial.println("EMERGENCY STOP: wing danger during RIGHT turn");
      return false;
    }

    delay(SAFE_STEP_MS);
  }

  MOTOR_1.stop();
  MOTOR_2.stop();
  return true;
}

bool MOVE_FORWARD_RETURN_SAFE(int pwmValue, unsigned long durationMs) { // FUNCTION TO MOVE THE CAR FORWARD SAFELY WHILE RETURNING HOME

  // AFTER THE TURNING MANEUVER, I USE THIS FUNCTION TO MOVE THE CAR FORWARD PAST THE OBSTACLE.
  // EVEN DURING THIS FORWARD MOVEMENT, I CONTINUE CHECKING THE FRONT AND SIDE DISTANCES
  // SO THE CAR CAN STOP AGAIN IF A NEW DANGER APPEARS.
  unsigned long start = millis();

  while (millis() - start < durationMs) {
    MOTOR_1.pwm = pwmValue;
    MOTOR_1.front();

    MOTOR_2.pwm = pwmValue;
    MOTOR_2.front();

    UPDATE_FRONT_DISTANCE();

    // stop if front gets bad again
    if (FRONT_DISTANCE > 0 && FRONT_DISTANCE <= 25) {
      STOP_RETURN_MOTION();
      Serial.println("EMERGENCY STOP: front obstacle during forward bypass");
      return false;
    }

    // also stop if either wing gets too close
    if (ANY_WING_DANGER()) {
      STOP_RETURN_MOTION();
      Serial.println("EMERGENCY STOP: wing danger during forward bypass");
      return false;
    }

    delay(SAFE_STEP_MS);
  }

  MOTOR_1.stop();
  MOTOR_2.stop();
  return true;
}
