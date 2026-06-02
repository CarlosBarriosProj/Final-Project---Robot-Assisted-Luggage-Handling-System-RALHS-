// CARLOS ANTONIO BARRIOS GUZMAN || FINAL PROJECT || 19/04/2026 || SECOND BRAIN (LCD DISPLAY, TRANSMITTER AND RECEIVER, ETC)

#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal.h>
#include <esp_wifi.h>


// ======= LCD SETUP =======

LiquidCrystal lcd(13, 18, 14, 27, 26, 25);

// LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

// ============================================================



// ======= DEFINE INDICATOR PINS =======

#define AVAILABLE_INDICATOR 4
#define OCCUPIED_INDICATOR 5

#define BUILT_IN 2

// ============================================================



// ======= DECLARE INT VARIABLES =======

int OBSTACLE_LEFT = -1;
int OBSTACLE_RIGHT = -1;

volatile int NUMBER_TAPS = 0;
int lastNumberTaps = -1;

// ============================================================



// ======= DECLARE BOOL VARIABLES =======

bool LEFT_SENSOR = false;
bool RIGHT_SENSOR = false;
volatile bool newLcdData = false;

// ============================================================



// ======= DEFINE ULTRASONIC SENSOR PINS =======

#define ECHO_LEFT 19
#define TRIG_LEFT 21

#define ECHO_RIGHT 22
#define TRIG_RIGHT 23

// ============================================================



// ======= MAC ADDRESS OF RECEIVER (ESP-NOW PROTOCOL) =======

uint8_t BROADCAST_ADD_1[] = {0x44, 0x1D, 0x64, 0xF9, 0x73, 0x1C};

// ============================================================



// ======= ESP-NOW SEND DISTANCE OF ULTRASONIC SENSORS =======

typedef struct __attribute__((packed)) DISTANCE_OBSTACLE {
  int ULTRA_DISTANCE_1;
  int ULTRA_DISTANCE_2;
} DISTANCE_OBSTACLE;

DISTANCE_OBSTACLE Obstacle_Data;

// ============================================================



// ======= ESPNOW CHANNEL =======

#define ESPNOW_CHANNEL 6

// ============================================================



// ======= RECEIVE TAPS =======

typedef struct __attribute__((packed)) TAPS_RECEIVED {
  volatile int TAPS;
} TAPS_RECEIVED;

TAPS_RECEIVED myTaps;

// ============================================================



// ======= RECEIVE LCD TEXT =======

typedef struct __attribute__((packed)) MESSAGE_TEXT {
  char line1[17];
  char line2[17];
} MESSAGE_TEXT;

MESSAGE_TEXT incomingMessage;

// ============================================================



// ======= LCD HELPER FUNCTION =======

void SHOW_LCD(const char* line1, const char* line2) { // FUNCTION TO DISPLAY A MESSAGE ON THE 16x2 LCD

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// ============================================================



// ======= ESP-NOW CALLBACK FUNCTION =======

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { // FUNCTION TO RECEIVE EITHER THE NUMBER OF TAPS OR A NEW LCD MESSAGE THROUGH ESP-NOW

  if (len == sizeof(TAPS_RECEIVED)) {
    memcpy(&myTaps, data, sizeof(myTaps));
    NUMBER_TAPS = myTaps.TAPS;

    Serial.print("ESP-NOW received TAPS = ");
    Serial.println(NUMBER_TAPS);
  }
  else if (len == sizeof(MESSAGE_TEXT)) {
    memcpy(&incomingMessage, data, sizeof(incomingMessage));
    newLcdData = true;
  }
  else {
    Serial.printf("Unknown packet size: %d\n", len);
  }
}

// ============================================================



// ======= FUNCTION PROTOTYPES =======

int GET_LEFT_DISTANCE();
int GET_RIGHT_DISTANCE();
void SEND_ULTRA_DISTANCE();
void UPDATE_STATUS_LEDS();
void LED_PATTERN_RETURN();
void INDICATOR_SENSORS();

// ============================================================



void setup() {

// ======= BEGIN SERIAL, LCD, AND PIN SETUP =======

  Serial.begin(115200);

  pinMode(AVAILABLE_INDICATOR, OUTPUT);
  pinMode(OCCUPIED_INDICATOR, OUTPUT);

  lcd.begin(16, 2);
  SHOW_LCD("- Waiting For -", "Authorized ID..");

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(BUILT_IN, OUTPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

// ============================================================



// ======= WIFI AND ESP-NOW SETUP =======

  WiFi.mode(WIFI_STA);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Receiver WiFi channel: ");
  Serial.println(WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    SHOW_LCD("ESP-NOW Failed", "Init Error");
    return;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, BROADCAST_ADD_1, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
  } else {
    Serial.println("ESP-NOW peer added successfully");
  }

  esp_now_register_recv_cb(onRecv);
  Serial.println("Receiver ready");

// ============================================================

}

void loop() {

  int tapsSnapshot = NUMBER_TAPS;

// ======= RETURN MODE SECTION =======
// U ONLY SEND THE LEFT AND RIGHT ULTRASONIC READINGS WHILE THE CAR IS IN RETURN MODE
// RHIS HELPS THE CAR TO DECIDE HOW TO AVOID OBSTACLES WHILE GOING BACK HOME

  if (tapsSnapshot == 4) {
    OBSTACLE_LEFT = GET_LEFT_DISTANCE();
    OBSTACLE_RIGHT = GET_RIGHT_DISTANCE();
    SEND_ULTRA_DISTANCE();
    INDICATOR_SENSORS();
  }

// ======= LCD MESSAGE UPDATE SECTION =======
// HERE I UPDATE THE LCD ONLY WHEN A NEW MESSAGE IS RECEIVED THROUGH ESP-NOW
// THIS AVOIDS REWRITING THE LCD CONTINOUSLY WHEN NOTHING HAS CHANGED

  if (newLcdData) {
    newLcdData = false;

    SHOW_LCD(incomingMessage.line1, incomingMessage.line2);

    Serial.println("LCD MESSAGE:");
    Serial.println(incomingMessage.line1);
    Serial.println(incomingMessage.line2);
  }

// ======= LOCAL LCD STATE SECTION =======
// HERE I CHANGE THE LCD SCREEN ONLY WHEN THE NUMBER_TAPS CHNAGES
// I USED THIS SO THE SCREEN REFLECTS THE CURRENT STATE OF THE SYSTEM WITHOUT FLICKERING

  if (tapsSnapshot != lastNumberTaps) {
    lastNumberTaps = tapsSnapshot;

    if (tapsSnapshot == 0) {
      SHOW_LCD("- Waiting For -", "Authorized ID..");
    }
    else if (tapsSnapshot == 3) {
      SHOW_LCD("- Calibrating! -", " Facing Home... ");
    }
    else if (tapsSnapshot == 4) {
      SHOW_LCD("-- Going Back --", "To Home Position");
    }
    else if (tapsSnapshot == 5) {
      SHOW_LCD("Final Alignment", " In Progress... ");
    }
  }

// ======= STATUS LED UPDATE =======

  UPDATE_STATUS_LEDS();
}

// ============================================================



// ===================== LED CONTROL =====================

void UPDATE_STATUS_LEDS() { // FUNCTION TO UPDATE THE AVAILABLE AND OCCUPIED INDICATORS BASED ON THE CURRENT SYSTEM STATE

  if (NUMBER_TAPS == 0) {
    digitalWrite(AVAILABLE_INDICATOR, HIGH);
    digitalWrite(OCCUPIED_INDICATOR, LOW);
  }
  else if (NUMBER_TAPS == 1 || NUMBER_TAPS == 2) {
    digitalWrite(OCCUPIED_INDICATOR, HIGH);
    digitalWrite(AVAILABLE_INDICATOR, LOW);
  }
  else if (NUMBER_TAPS == 3 || NUMBER_TAPS == 4) {
    LED_PATTERN_RETURN();
  }
  else {
    digitalWrite(AVAILABLE_INDICATOR, LOW);
    digitalWrite(OCCUPIED_INDICATOR, LOW);
  }
}

void LED_PATTERN_RETURN() { // FUNCTION TO CREATE A BLINKING LED PATTERN WHILE THE CAR IS RETURNING OR TURNING HOME

  unsigned long currentMillis = millis();
  static unsigned long ledTimer = 0;
  static int ledState = 0;

  if (currentMillis - ledTimer > 500) {
    ledTimer = currentMillis;
    ledState++;

    switch (ledState) {
      case 1:
        digitalWrite(OCCUPIED_INDICATOR, LOW);
        digitalWrite(AVAILABLE_INDICATOR, HIGH);
        break;

      case 2:
        digitalWrite(OCCUPIED_INDICATOR, HIGH);
        digitalWrite(AVAILABLE_INDICATOR, LOW);
        break;

      case 3:
        digitalWrite(OCCUPIED_INDICATOR, LOW);
        digitalWrite(AVAILABLE_INDICATOR, HIGH);
        break;

      case 4:
        digitalWrite(OCCUPIED_INDICATOR, HIGH);
        digitalWrite(AVAILABLE_INDICATOR, LOW);
        ledState = 0;
        break;
    }
  }
}

// ============================================================



// ======= ULTRASONIC SENSOR FUNCTIONS =======

int GET_LEFT_DISTANCE() { // FUNCTION TO MEASURE THE DISTANCE DETECTED BY THE LEFT ULTRASONIC SENSOR

  digitalWrite(TRIG_LEFT, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_LEFT, LOW);

  unsigned long DURATION_LEFT = pulseIn(ECHO_LEFT, HIGH, 30000);

  if (DURATION_LEFT == 0) {
    LEFT_SENSOR = false;
    return -1;
  }

  int LEFT_DISTANCE = DURATION_LEFT * 0.034 / 2;
  LEFT_SENSOR = true;
  return LEFT_DISTANCE;
}

int GET_RIGHT_DISTANCE() { // FUNCTION TO MEASURE THE DISTANCE DETECTED BY THE RIGHT ULTRASONIC SENSOR

  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_RIGHT, LOW);

  unsigned long DURATION_RIGHT = pulseIn(ECHO_RIGHT, HIGH, 30000);

  if (DURATION_RIGHT == 0) {
    RIGHT_SENSOR = false;
    return -1;
  }

  int RIGHT_DISTANCE = DURATION_RIGHT * 0.034 / 2;
  RIGHT_SENSOR = true;
  return RIGHT_DISTANCE;
}

void SEND_ULTRA_DISTANCE() { // FUNCTION TO SEND THE LEFT AND RIGHT ULTRASONIC DISTANCES TO THE MAIN CAR THROUGH ESP-NOW

  Obstacle_Data.ULTRA_DISTANCE_1 = OBSTACLE_LEFT;
  Obstacle_Data.ULTRA_DISTANCE_2 = OBSTACLE_RIGHT;

  esp_err_t RESULT_1 = esp_now_send(BROADCAST_ADD_1, (uint8_t*)&Obstacle_Data, sizeof(Obstacle_Data));

  if (RESULT_1 == ESP_OK) {
    Serial.printf("Sent -> DISTANCE_LEFT: %d || DISTANCE_RIGHT: %d\n",
                  Obstacle_Data.ULTRA_DISTANCE_1,
                Obstacle_Data.ULTRA_DISTANCE_2);
  } else {
    Serial.print("RESULT_1: ");
    Serial.println((int)RESULT_1);
  }
}

void INDICATOR_SENSORS() { // FUNCTION TO USE THE BUILT-IN LED TO SHOW IF THE LEFT SENSOR, RIGHT SENSOR, OR BOTH SENSORS ARE ACTIVE

  if (RIGHT_SENSOR && !LEFT_SENSOR) {

    digitalWrite(BUILT_IN, HIGH);
    delay(50);
    digitalWrite(BUILT_IN, LOW);
    delay(50);

  }
  else if (!RIGHT_SENSOR && LEFT_SENSOR) {

    digitalWrite(BUILT_IN, HIGH);
    delay(500);
    digitalWrite(BUILT_IN, LOW);
    delay(500);

  }
  else if (RIGHT_SENSOR && LEFT_SENSOR) {

    digitalWrite(BUILT_IN, HIGH);

  }
  else {

    digitalWrite(BUILT_IN, LOW);

  }
}