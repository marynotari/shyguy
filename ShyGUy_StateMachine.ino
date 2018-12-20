/*

  Shy Guy - Medians - State Machine
  12/04/2018 – 12/08/2018
  Mary Notari

  Based on:
  https://github.com/DrGFreeman/SharpDistSensor
  MIT License
  Copyright (c) 2017 Julien de la Bruere-Terreault <drgfreeman@tuta.io>

  This program creates Shy Guy, a micro servo that moves
  according to the position of the user's hand in relation
  to an Infrared Proximity Sensor.

  Circuit:
  - IR Sensor is attached to PIN A0, 5V, and GROUND
  - Micro Servo is attached to PIN 9, 5V, and GROUND

  Shy Guy has three emotional states:
    Fear
    Curiosity
    Affection

  FEAR:
  Shy Guy moves away from the hand as it gets closer.

  -- If the user keeps their hand still for 5 seconds and
  within 10-15cm away from the sensor...

  CURIOSITY:
  ...Shy Guy moves forward aincrementally.

  -- If the user moves their hand beyond 10-15cm,
  Shy Guy reverts to FEAR.

  -- If the user keeps their hand still until Shy Guy gets close enough...

  AFFECTION:
  Shy Guy will follow the hand forward, but not back, allowing the hand to
  pet it.

*/

/* SERVO **************************/
#include <Servo.h>
Servo myServo;      // TowerPro Micro Servo 99 SG90
int servoAngle;
int startingAngle = 40;
int minAngle = 15;
int maxAngle = 165;
//unsigned int servoAngleFEAR;
//unsigned int servoAngleCURIOSITY;
//unsigned int servoAngleAFFECTION;

/* IR SENSOR **********************/
#include <SharpDistSensor.h>

// Sharp IR GP2Y0A41SK0F (4-30cm, analog)
// Analog pin to which the sensor is connected
const byte sensorPin = A0;

// Window size of the median filter (odd number 1-655, 1 = no filtering)
// Larger windows means slower processing time
const byte medianFilterWindowSize = 5;

// Create an object instance of the SharpDistSensor class
SharpDistSensor sensor(sensorPin, medianFilterWindowSize);


/* QUICKSTATS *********************/
// QuickStats is a library to help do math to sensor readings
// We use here to smooth the servo movements by finding the
// median of the IR sensor readings
#include <QuickStats.h>
QuickStats stats; // initialize an instance of this class

// Define the number of readings to calculate the median from.
// The higher the number, the smoother the servo, but the slower
// it will respond to the hand. Using const to make it read only.
const int numOfReadings = 5;

float readings[numOfReadings];  // an array to hold the readings from the sensor
float medianDist;               // variable for holding median of the readings
int readingsIndex = 0;          // the index of the current reading


/* STATE MACHINE VARIABLES ******************/
boolean handIsInRange = false;        // keep track of whether the hand is in maximum range
boolean shyGuyIsCurious = false;      // keep track of Shy Guy's emotional state
boolean shyGuyIsAffectionate = false;
unsigned long happyZoneCounter = 0;   // keep track of how long the hand has been in the HAPPY ZONE
int handIsInHappyZone = 0;            // keep track of whether the hand is in the HAPPY ZONE

int minDist = 40; // in millimeters
int maxDist = 300;
int minHappyZoneDist = 100;
int maxHappyZoneDist = 120;

/* RUNS ONCE **********************/
void setup() {

  // Attach servo to PIN 9 and start Serial Monitor
  myServo.attach(9);            // servo receives data from pin 9
  Serial.begin(9600);
  Serial.println("Reading..."); // this let's us know if Serial has begun

  // This sets the model of sensor that the library is calibrated to:
  // The default is GP2Y0A60SZLF_5V
  sensor.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

  // Initialize the array of readings:
  for (int thisReading = 0; thisReading < numOfReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  //start the servo at 40 degrees:
  myServo.write(startingAngle);

}

/* RUNS CONTINUOUSLY *************/
void loop() {

  // Get distance from sensor
  unsigned int distance = sensor.getDist();

  // Assign current distance to the current index in the array of readings
  readings[readingsIndex] = distance;

  // Advance to the next position in the array:
  readingsIndex = readingsIndex + 1;

  // If we're at the end of the array...
  if (readingsIndex >= numOfReadings) {
    // ...wrap around to the beginning:
    readingsIndex = 0;
  }

  // Calculate the median distance:
  medianDist = stats.median(readings, numOfReadings); // Median filter from QuickStats.h line 25

  // Map the distance to the servo angle
  // The smaller the distance, the larger the angle
  // The servo can get jittery when pushed to its limits so I've limited it
  servoAngle = map(medianDist, minDist, maxDist, maxAngle, startingAngle);


  /* FEAR MODE ********/
  // If Hand is within maximum range, Shy Guy interacts with FEAR.
  if (medianDist >= minDist && medianDist <= maxDist) {

    // Note that Hand is within the min/max range
    handIsInRange = true;
    Serial.println("in range"); //debugging

    Serial.print("Distance in MM: ");
    Serial.print(medianDist);
    Serial.print(", ");
    Serial.print(servoAngle);
    Serial.println(" degrees");

    myServo.write(servoAngle);
    delay(50); // wait the time it takes to move the servo plus the time to take the median (22ms)

    /* CURIOSITY MODE *********/
    // if hand is in happy zone...
    if (medianDist >= minHappyZoneDist && medianDist <= maxHappyZoneDist && !shyGuyIsAffectionate) {

      //...start the counter...
      happyZoneCounter++;

      Serial.println(happyZoneCounter); //debugging

      //...if the hand remains in the happy zone for 5 seconds and Shy Guy is not in AFFECTION...
      if (happyZoneCounter >= 100 && !shyGuyIsAffectionate) {

        shyGuyIsCurious = true;
        Serial.println("curious"); //debugging

        // ...move the servo incrementally.
        // servo makes quick small move
        servoAngle = servoAngle - random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // wait 2 sec
        delay(2000);

        // servo makes another quick small move
        servoAngle = servoAngle - random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // wait
        delay(500);

        // servo makes another quick small move
        servoAngle = servoAngle - random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // wait
        delay(1000);

        // servo makes another quick small move
        servoAngle = servoAngle - random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // servo makes another quick small move
        servoAngle = servoAngle - random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // wait
        delay(3000);

        // servo makes another quick small move
        servoAngle = servoAngle + random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // wait
        delay(3000);

        // servo makes another quick small move
        servoAngle = servoAngle - random(2, 20);
        myServo.write(servoAngle);

        Serial.print("Distance in MM: ");
        Serial.print(medianDist);
        Serial.print(", ");
        Serial.print(servoAngle);
        Serial.println(" degrees");

        // wait
        delay(2000);

        // servo makes a slower move
        for (servoAngle = servoAngle; servoAngle >= startingAngle; servoAngle -= 1) {
          myServo.write(servoAngle);

          Serial.print("Distance in MM: ");
          Serial.print(medianDist);
          Serial.print(", ");
          Serial.print(servoAngle);
          Serial.println(" degrees");

          delay(50);
        }

        Serial.println("starting angle"); //debugging

        shyGuyIsAffectionate = true;

      }
    }
    // if hand moves outside of happy zone...
    else {
      //...servo resets to FEAR mode
      happyZoneCounter = 0;
      shyGuyIsCurious = false;
      Serial.println("afraid"); //debugging
    }

    /* AFFECTION MODE ********/
    // once Shy Guy gets to his starting angle, he interacts with AFFECTION,
    // and stays in that mode until the hand leaves the range
    while (servoAngle < startingAngle && shyGuyIsAffectionate) {
      
      // Get distance from sensor
      distance = sensor.getDist();

      // Assign current distance to the current index in the array of readings
      readings[readingsIndex] = distance;

      // Advance to the next position in the array:
      readingsIndex = readingsIndex + 1;

      // If we're at the end of the array...
      if (readingsIndex >= numOfReadings) {
        // ...wrap around to the beginning:
        readingsIndex = 0;
      }

      // Calculate the median distance:
      medianDist = stats.median(readings, numOfReadings); // Median filter from QuickStats.h line 25

      // follow hand forward only
      servoAngle = map(medianDist, minDist, maxDist, startingAngle - 10, 15);
      /* put in code that will keep Shy Guy from moving backwards*/
      myServo.write(servoAngle);

      Serial.println("love"); //debugging

      Serial.print("Distance in MM: ");
      Serial.print(medianDist);
      Serial.print(", ");
      Serial.print(servoAngle);
      Serial.println(" degrees");

      /* put in code that will make it reset only after hand moves out of range */
      /* or put in code that will make Shy Guy stay still even if hand moves out of range,
         in which case, the reset would have to be manual */
    }
  }
  // if hand moves outside of range, servo resets to starting angle
  else {
    myServo.write(startingAngle);
    shyGuyIsAffectionate = false;

    //    Serial.println("...goodbye.");
  }

}
