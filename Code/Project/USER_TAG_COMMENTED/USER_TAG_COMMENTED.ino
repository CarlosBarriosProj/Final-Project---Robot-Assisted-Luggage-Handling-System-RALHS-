// CARLOS ANTONIO BARRIOS GUZMAN || FINAL PROJECT || 19/04/2026 || TAG USER (MOVING TAG)

/**
 * Ultra WideBand Real-Time Positioning System (UWBRTLS)
 */

#define IS_TAG
// #define IS_ANCHOR

bool START_UP = true;

#include <SPI.h>
#include <DW1000Ranging.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>   // ADDED FOR FIXED CHANNEL

// ===================== ESP-NOW =====================
uint8_t BROADCAST_ADD[] = {0x44, 0x1D, 0x64, 0xF9, 0x73, 0x1C};

// SEND BOTH ANCHOR DISTANCES TOGETHER
typedef struct __attribute__((packed)) STRUCT_MESSAGE_SEND {
  float DIST_84;
  float DIST_85;
} STRUCT_MESSAGE_SEND;

STRUCT_MESSAGE_SEND myData;

// ===================== OLED SETUP =====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===================== UWB SETUP =====================
#define DEVICE_ADDRESS "02:00:00:00:00:00:00:02"

volatile int NUMBER_TAPS = 0;

int lastTapState = -1;

// ===================== RECEIVE NUMBER OF TAPS =====================

typedef struct __attribute__((packed)) MESSAGE_RECEIVED {

  int TAPS;

} MESSAGE_RECEIVED;

MESSAGE_RECEIVED TapsData;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(MESSAGE_RECEIVED)) {
    Serial.printf("Wrong packet size: %d\n", len);
    return;
  }

  memcpy(&TapsData, data, sizeof(TapsData));
  NUMBER_TAPS = TapsData.TAPS;

  Serial.print("ESP-NOW received TAPS = ");
  Serial.println(NUMBER_TAPS);
}

char shortAddress[6];

// ===================== ANCHOR IDS =====================
const uint16_t ANCHOR_1_ID = 0x0084;
const uint16_t ANCHOR_2_ID = 0x0085;

// ===================== TIMING =====================
const unsigned long UPDATE_INTERVAL_MS = 300; // TIME BETWEEN SCREEN UPDATES

// ===================== FIXED ESP-NOW CHANNEL =====================
// THIS CHANNEL MUST MATCH THE RECEIVER WIFI CHANNEL
const uint8_t ESPNOW_CHANNEL = 6;

// ===================== FUNCTION PROTOTYPES =====================
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.display();

  SPI.begin(18, 19, 23);

  DW1000Ranging.initCommunication(27, 4, 34);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

#ifdef IS_ANCHOR
  DW1000Ranging.startAsAnchor(DEVICE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
#elif defined(IS_TAG)
  DW1000Ranging.startAsTag(DEVICE_ADDRESS, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  // FORCE THE SENDER TO USE THE SAME CHANNEL AS THE RECEIVER FOR ESP-NOW
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Sender WiFi channel: ");
  Serial.println(WiFi.channel());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW failed to init");
  } else {
    esp_now_register_recv_cb(onRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_ADD, 6);
    peerInfo.channel = ESPNOW_CHANNEL;   // MUST MATCH RECEIVER CHANNEL
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer");
    } else {
      Serial.println("ESP-NOW peer added successfully");
    }
  }
#endif

  byte* currentShortAddress = DW1000Ranging.getCurrentShortAddress();
  sprintf(shortAddress, "%02X%02X", currentShortAddress[1], currentShortAddress[0]);

  Serial.print("Device short address: ");
  Serial.println(shortAddress);

  Serial.println("Setup complete");
}

void loop() {
  DW1000Ranging.loop();
}

// ===================== MAIN LOGIC =====================
void newRange() {

  // THIS FUNCTION IS THE MAIN PART OF THE TAG LOGIC.
  // HERE I READ THE DISTANCE COMING FROM BOTH ANCHORS,
  // SMOOTH THE VALUES WITH A SMALL MOVING AVERAGE BUFFER,
  // SHOW THE RESULTS ON THE OLED, AND SEND THEM THROUGH ESP-NOW
  // WHEN THE CURRENT STATE ALLOWS IT.
  if (START_UP) {
    display.clearDisplay();
    display.setTextSize(2);
    START_UP = false;
  }

  static float buffer84[5] = {0};
  static float buffer85[5] = {0};
  static int index84 = 0;
  static int index85 = 0;

  static unsigned long lastUpdate = 0;

  static float latestAvg84 = 0.0;
  static float latestAvg85 = 0.0;

  char buffer[21];
  uint16_t anchorID = DW1000Ranging.getDistantDevice()->getShortAddress();
  float distance = DW1000Ranging.getDistantDevice()->getRange();

  Serial.printf("anchorID = 0x%04X, raw distance = %.2f m\n", anchorID, distance);

  if (distance < 0.0){
    distance = 0.0;
  }

  // APPLY CORRECTION ONLY TO ANCHOR 0x0085
  if (anchorID == ANCHOR_2_ID) {
    distance -= 0.05;
  }

  if (distance < 0.0) {
    distance = 0.0;
  }

  // STORE EACH DISTANCE IN ITS CORRESPONDING BUFFER
  if (anchorID == ANCHOR_1_ID) {
    buffer84[index84] = distance;
    index84 = (index84 + 1) % 5;
  }
  else if (anchorID == ANCHOR_2_ID) {
    buffer85[index85] = distance;
    index85 = (index85 + 1) % 5;
  }
  else {
    Serial.printf("Unknown anchor ID: 0x%04X\n", anchorID);
    return;
  }

  // RECALCULATE BOTH AVERAGES EVERY TIME A NEW RANGE IS RECEIVED
  latestAvg84 = 0.0;
  for (int i = 0; i < 5; i++) latestAvg84 += buffer84[i];
  latestAvg84 /= 5.0;

  latestAvg85 = 0.0;
  for (int i = 0; i < 5; i++) latestAvg85 += buffer85[i];
  latestAvg85 /= 5.0;

  // ONLY UPDATE THE SCREEN AFTER THE SELECTED INTERVAL
  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) return;
  lastUpdate = millis();


// ===================== OLED DISPLAY =====================
display.clearDisplay();

display.setCursor(0, 0);
snprintf(buffer, sizeof(buffer), "0084");
display.print(buffer);

display.setCursor(50, 0);
snprintf(buffer, sizeof(buffer), "%.2f m", latestAvg84);
display.print(buffer);

display.setCursor(0, 24);
snprintf(buffer, sizeof(buffer), "0085");
display.print(buffer);

display.setCursor(50, 24);
snprintf(buffer, sizeof(buffer), "%.2f m", latestAvg85);
display.print(buffer);

display.setCursor(0, 50);
snprintf(buffer, sizeof(buffer), "TAPS: %d", NUMBER_TAPS);
display.print(buffer);

display.display();

static unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 200;

if (NUMBER_TAPS != lastTapState) {
  lastTapState = NUMBER_TAPS;
  Serial.print("NUMBER_TAPS changed to: ");
  Serial.println(NUMBER_TAPS);
}

if (NUMBER_TAPS <= 2 && millis() - lastSendTime >= SEND_INTERVAL_MS) {
  lastSendTime = millis();

#ifdef IS_TAG
  // IN THESE STATES THE TAG IS ALLOWED TO SEND DISTANCES TO THE MAIN CONTROLLER
  myData.DIST_84 = latestAvg84;
  myData.DIST_85 = latestAvg85;

  esp_err_t result = esp_now_send(BROADCAST_ADD, (uint8_t*)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.printf("Sent -> 0084: %.2f m | 0085: %.2f m\n",
                  myData.DIST_84, myData.DIST_85);
  } else {
    Serial.print("ESP-NOW send error: ");
    Serial.println((int)result);
  }
#endif
}
else if (NUMBER_TAPS > 2) {
  // AFTER THIS POINT THE SYSTEM CHANGES MODE, SO THIS TAG SHOULD NOT KEEP SENDING
  Serial.println("TAG_1 not allowed to send right now");

 }
}

// ===================== CALLBACKS =====================
void newDevice(DW1000Device *device) {
  // THIS FUNCTION HELPS ME SEE WHEN A NEW UWB DEVICE IS DETECTED
  Serial.print("New device detected: 0x");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  // THIS FUNCTION HELPS ME KNOW WHEN A DEVICE BECOMES INACTIVE
  Serial.print("Inactive device: 0x");
  Serial.println(device->getShortAddress(), HEX);
}
