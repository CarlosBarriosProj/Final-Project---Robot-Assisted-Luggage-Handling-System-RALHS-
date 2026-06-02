// RALHS FINAL PROJECT || 19/04/2026 || ANCHOR 1 (LEFT SIDE)

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <math.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <WiFi.h>
#include <esp_now.h>


// ======= ANCHOR ADDRESS AND TAG IDS =======

#define ANCHOR_ADD "85:00:5B:D5:A9:9A:E2:9D"

const uint16_t TAG_1 = 0x0002;
const uint16_t TAG_2 = 0x0004;

uint16_t TARGET_TAG = 0;

uint16_t Adelay = 16621;

volatile int NUMBER_TAPS = 0;

int lastTapState = -1;

// ============================================================



// ======= SPI AND DWM1000 PIN SETUP =======

#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// ============================================================



// ======= FILTERING VARIABLES =======

#define N 7
float buf[N];
int buf_i = 0;
int buf_count = 0;

float ema = -1.0f;
const float alpha = 0.18f;

const float minRange = 0.15f;
const float maxRange = 8.0f;

const float outlierFromMedian = 0.20f;
const float printDeadband = 0.01f;

float lastPrinted = -1.0f;
unsigned long lastGoodSampleMs = 0;
const unsigned long staleTimeoutMs = 500;

// ============================================================



// ======= RECEIVE NUMBER OF TAPS =======
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

  if (NUMBER_TAPS != lastTapState) {
    
    lastTapState = NUMBER_TAPS;


    if (NUMBER_TAPS < 3){
    TARGET_TAG = TAG_1;
    Serial.println("Following TAG_1"); }

    else if (NUMBER_TAPS == 3){
    TARGET_TAG = 0x0000;
    RESET_TRACK_ID();
    Serial.println("Tracking Cleared");
    Serial.println("Tracking Nobody");}

    else if (NUMBER_TAPS == 4){
    TARGET_TAG = TAG_2;
    RESET_TRACK_ID();
    Serial.print("Now tracking new TARGET_TAG = 0x");
    Serial.println(TARGET_TAG, HEX);

  }
 }
}

// ============================================================



// ======= MEDIAN FILTER FUNCTION =======
// THIS FUNCTION HELPS ME REDUCE NOISE BY SORTING THE LAST N SAMPLES
// AND TAKING THE MIDDLE VALUE BEFORE APPLYING THE EMA FILTER.
float medianN(const float *arr, int n) {
  float tmp[N];
  for (int i = 0; i < n; i++) tmp[i] = arr[i];

  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }
  return tmp[n / 2];
}

void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void RESET_TRACK_ID();

void setup() {

  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  delay(200);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);   // SET THE RECEIVER TO THE SAME WIFI CHANNEL USED BY THE SENDER
  esp_wifi_set_promiscuous(false);

  uint8_t ch;
  wifi_second_chan_t second_ch;
  esp_wifi_get_channel(&ch, &second_ch);

  Serial.print("Receiver WiFi channel: ");
  Serial.println(ch);

  if (esp_now_init() != ESP_OK) {
  Serial.println("ESP-NOW init failed");
  return;
}

 esp_now_register_recv_cb(onRecv);
 Serial.println("ESP-NOW Receiver Ready");


  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);


  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}

void loop() {
  
  // THIS LOOP KEEPS THE ANCHOR LISTENING FOR UWB RANGE UPDATES ALL THE TIME.
  DW1000Ranging.loop();

  // HERE I CHECK IF THE LAST VALID SAMPLE IS TOO OLD.
  if (lastGoodSampleMs > 0 && millis() - lastGoodSampleMs > staleTimeoutMs) {
    // OPTIONAL: REPORT STALE DATA
    // Serial.println("STALE");
  }
}

void newRange() {

  // THIS FUNCTION IS THE MAIN RANGING SECTION.
  // HERE I ONLY PROCESS THE DISTANCE OF THE TAG THAT SHOULD BE TRACKED
  // ACCORDING TO THE CURRENT NUMBER_TAPS STATE SENT THROUGH ESP-NOW.
  DW1000Device *dev = DW1000Ranging.getDistantDevice();

  if (!dev) return;

  uint16_t shortAddr = dev->getShortAddress();

  if (shortAddr != TARGET_TAG) return;

   float d = dev->getRange();

  // FIRST I REJECT DISTANCES THAT ARE OUTSIDE THE VALID OPERATING RANGE.
  if (d < minRange || d > maxRange) return;

  // THEN I STORE THE NEW SAMPLE IN A SMALL BUFFER SO I CAN APPLY A MEDIAN FILTER.
  buf[buf_i] = d;
  buf_i = (buf_i + 1) % N;
  if (buf_count < N) buf_count++;

  if (buf_count < N) return;

  float med = medianN(buf, N);

  // HERE I REMOVE STRONG OUTLIERS BY COMPARING THE CURRENT SAMPLE TO THE MEDIAN.
  if (fabs(d - med) > outlierFromMedian) return;

  // AFTER THAT I APPLY AN EXPONENTIAL MOVING AVERAGE TO MAKE THE DISTANCE MORE STABLE.
  if (ema < 0) ema = med;
  ema = alpha * med + (1.0f - alpha) * ema;

  lastGoodSampleMs = millis();

  // THIS PREVENTS PRINTING VERY SMALL CHANGES THAT DO NOT REALLY MATTER.
  if (lastPrinted > 0 && fabs(ema - lastPrinted) < printDeadband) return;
  lastPrinted = ema;

  Serial.print("ID:");
  Serial.print(shortAddr, HEX);
  Serial.print(",RAW:");
  Serial.print(d, 3);
  Serial.print(",MED:");
  Serial.print(med, 3);
  Serial.print(",EMA:");
  Serial.println(ema, 3);

  Serial.print("NUMBER_TAPS = ");
  Serial.println(NUMBER_TAPS);
}

void newDevice(DW1000Device *device) {

  // THIS FUNCTION LETS ME KNOW WHEN THE CURRENT TARGET TAG BECOMES ACTIVE.
  if (!device) return;

  if (device->getShortAddress() != TARGET_TAG) return;

  Serial.print("NEW:");
  Serial.println(device->getShortAddress(), HEX);

}

void inactiveDevice(DW1000Device *device) {

  // THIS FUNCTION LETS ME KNOW WHEN THE CURRENT TARGET TAG IS LOST OR BECOMES INACTIVE.
  if (!device) return;

  if (device->getShortAddress() != TARGET_TAG) return;

  Serial.print("LOST:");
  Serial.println(device->getShortAddress(), HEX);
}

void RESET_TRACK_ID() {
  // EVERY TIME I CHANGE THE TRACKED TAG OR CLEAR TRACKING,
  // I RESET THE FILTER VARIABLES SO OLD DISTANCE DATA DOES NOT AFFECT THE NEW TAG.
  buf_i = 0;
  buf_count = 0;
  ema = -1.0f;
  lastPrinted = -1.0f;
  lastGoodSampleMs = 0;

  for (int i = 0; i < N; i++) {
    buf[i] = 0.0f;
  }
}
