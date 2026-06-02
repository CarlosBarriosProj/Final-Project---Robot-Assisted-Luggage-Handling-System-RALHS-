// CARLOS ANTONIO BARRIOS GUZMAN || FINAL PROJECT || 19/04/2026 || CODE TO CALIBRATE ANCHOR 2 

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
char this_anchor_addr[] = "84:00:5B:D5:A9:9A:E2:9D";
float this_anchor_target_distance = 2.0; // MEASURED DISTANCE IN METERS

uint16_t this_anchor_Adelay = 16627;   // 16625, 16632, 16627, 16625
uint16_t Adelay_delta = 100;           // INITIAL BINARY SEARCH STEP SIZE

static float last_delta = 0.0;

// ===================== AVERAGING VARIABLES =====================
// THESE VARIABLES ARE USED TO AVERAGE MULTIPLE DISTANCE READINGS
// BEFORE ADJUSTING THE ANTENNA DELAY VALUE
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
  // THIS FUNCTION PERFORMS THE ANTENNA DELAY CALIBRATION.
  // IT COLLECTS MULTIPLE DISTANCE READINGS, COMPUTES AN AVERAGE,
  // AND ADJUSTS THE ANTENNA DELAY UNTIL THE MEASURED DISTANCE MATCHES THE REAL ONE.

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

  // CALCULATE THE AVERAGE DISTANCE AFTER 100 SAMPLES
  float avgDist = distanceSum / NUM_SAMPLES;

  Serial.println();
  Serial.print("Average distance over ");
  Serial.print(NUM_SAMPLES);
  Serial.print(" samples: ");
  Serial.println(avgDist);

  // THIS VALUE REPRESENTS THE ERROR BETWEEN MEASURED AND REAL DISTANCE
  float this_delta = avgDist - this_anchor_target_distance;

  // STOP CALIBRATION WHEN STEP SIZE IS VERY SMALL
  if (Adelay_delta < 3) {
    Serial.print("Final Adelay: ");
    Serial.println(this_anchor_Adelay);
    while (1); // DONE CALIBRATING
  }

  // IF ERROR CHANGES SIGN, REDUCE STEP SIZE FOR BETTER PRECISION
  if (this_delta * last_delta < 0.0) {
    Adelay_delta /= 2;
  }

  last_delta = this_delta;

  // ADJUST ANTENNA DELAY BASED ON ERROR DIRECTION
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

  // RESET AVERAGING VARIABLES FOR NEXT ITERATION
  sampleCount = 0;
  distanceSum = 0.0;
}

void newDevice(DW1000Device *device)
{
  // THIS FUNCTION INDICATES WHEN A NEW DEVICE IS DETECTED
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  // THIS FUNCTION INDICATES WHEN A DEVICE IS LOST OR INACTIVE
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}