// RALHS FINAL PROJECT || 19/04/2026 || CODE TO CALIBRATE ANCHOR 1 

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// ===================== SPI AND DWM1000 PIN SETUP =====================
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// ===================== ANCHOR CALIBRATION VALUES =====================
char this_anchor_addr[] = "10:00:5B:D5:A9:9A:E2:9D";
float this_anchor_target_distance = 2.0; // MEASURED DISTANCE IN METERS

uint16_t this_anchor_Adelay = 16621;   // 16616, 16626, 16618, 16625
uint16_t Adelay_delta = 100;           // INITIAL BINARY SEARCH STEP SIZE

static float last_delta = 0.0;

// ===================== AVERAGING VARIABLES =====================
// THESE VARIABLES HELP ME AVERAGE MANY READINGS BEFORE CHANGING THE ANTENNA DELAY
const int NUM_SAMPLES = 100;
int sampleCount = 0;
float distanceSum = 0.0;

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);

  Serial.print("Starting Adelay: ");
  Serial.println(this_anchor_Adelay);
  Serial.print("Measured distance: ");
  Serial.println(this_anchor_target_distance);

  DW1000.setAntennaDelay(this_anchor_Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.useRangeFilter(true);

  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
}

void loop()
{
  DW1000Ranging.loop();
}

void newRange()
{
  // THIS FUNCTION IS THE MAIN CALIBRATION LOGIC.
  // HERE I COLLECT MANY DISTANCE SAMPLES, COMPUTE THEIR AVERAGE,
  // COMPARE THAT AVERAGE TO THE REAL MEASURED DISTANCE,
  // AND THEN ADJUST THE ANTENNA DELAY UNTIL THE ERROR BECOMES VERY SMALL.
  float dist = DW1000Ranging.getDistantDevice()->getRange();

  distanceSum += dist;
  sampleCount++;

  Serial.print("Sample ");
  Serial.print(sampleCount);
  Serial.print(": ");
  Serial.println(dist);

  if (sampleCount < NUM_SAMPLES) {
    return;
  }

  // CALCULATE THE AVERAGE DISTANCE AFTER 100 READINGS
  float avgDist = distanceSum / NUM_SAMPLES;

  Serial.println();
  Serial.print("Average distance over ");
  Serial.print(NUM_SAMPLES);
  Serial.print(" samples: ");
  Serial.println(avgDist);

  // THIS DELTA TELLS ME HOW FAR THE MEASURED AVERAGE IS FROM THE REAL DISTANCE
  float this_delta = avgDist - this_anchor_target_distance;

  // WHEN THE STEP SIZE BECOMES VERY SMALL, I STOP THE CALIBRATION
  if (Adelay_delta < 3) {
    Serial.print("Final Adelay: ");
    Serial.println(this_anchor_Adelay);
    while (1); // DONE CALIBRATING
  }

  // IF THE ERROR CHANGES SIGN, IT MEANS I PASSED THE CORRECT VALUE,
  // SO I REDUCE THE STEP SIZE TO MAKE THE SEARCH MORE PRECISE
  if (this_delta * last_delta < 0.0) {
    Adelay_delta /= 2;
  }

  last_delta = this_delta;

  // IF THE MEASURED DISTANCE IS TOO LARGE, I INCREASE THE ANTENNA DELAY.
  // OTHERWISE, I DECREASE IT.
  if (this_delta > 0.0) {
    this_anchor_Adelay += Adelay_delta;
  } else {
    this_anchor_Adelay -= Adelay_delta;
  }

  DW1000.setAntennaDelay(this_anchor_Adelay);

  Serial.print("Updated Adelay: ");
  Serial.println(this_anchor_Adelay);
  Serial.print("Current step size: ");
  Serial.println(Adelay_delta);
  Serial.println("--------------------------------");

  // RESET THE AVERAGING VARIABLES FOR THE NEXT GROUP OF 100 SAMPLES
  sampleCount = 0;
  distanceSum = 0.0;
}

void newDevice(DW1000Device *device)
{
  // THIS FUNCTION HELPS ME KNOW WHEN A NEW DEVICE IS DETECTED
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  // THIS FUNCTION HELPS ME KNOW WHEN A DEVICE BECOMES INACTIVE
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
