/*
  Project Name:   Slumber
  Description:    Slumber is an Arduino 101 based module stored inside your
                  pillow. It tracks sleep movement, records sleep, provides immediate
                  feedback and improves the overall happiness/life satisfaction through 
                  a more informed nights sleep.
  Author:         Zach Alam
  Release Date:   28 June 2017
  Revision Date:  23 July 2017
  Version:        1.2
  ----
  Made with <3 by Zach Alam in Las Vegas, Nevada!
*/

#include <CurieIMU.h>       // Gyroscope LIB
#include <CurieBLE.h>       // BluetoothLE LIB


const int LED_PIN = 9;    // LED connected to digital pin 9 (with PWM)

// when DEBUG_MODE is true, logs are enabled.
const bool DEBUG_MODE = true;


// flags for types of sleep patterns.
const int SLEEPING_PATTERNS = 5;      // number of patterns to categorize.
const int SLEEPING_PATTERN_EXAMPLES = 10; // number of examples for each sleeping pattern.
const int PATTERN_SAMPLE_COUNT = 6;   // number of samples in each analysis
const int DIMENSION_COUNT = 3;        // x,y,z axis
// ---
const int PATTERN_NO_MOVEMENT = 0;
const int PATTERN_GENTLE_MOVEMENT = 1;
const int PATTERN_HEAD_MOVEMENT = 2;
const int PATTERN_RESTLESS_BODY = 3;
const int PATTERN_TOSSING_TURNING = 4;
const String PATTERN_NAMES[SLEEPING_PATTERNS] = {"No Movement", "Gentle Movement", 
"Head Movement", "Restless Body", "Tossing & Turning"};

const int PATTERN_MATCH_THRESHHOLD = 50;     // x percent matching threshold.
const int MATCHES_REQUIRED = 1; // max num: 5, signals the pattern matched.

// the sleepMatrix is a 2d array that analyzes a 500ms
// period of sleep for specific sleep patterns.
int sleepMatrix[PATTERN_SAMPLE_COUNT][DIMENSION_COUNT];

// flag that signified whether we're recording or not.
bool recordingActive = false;
int recordingNoMovements = 0;
int recordingTotalRecords = 0;

// init const int array with sleeping pattern data we gathered during sleep study.
const int SLEEP_DATA[SLEEPING_PATTERNS][SLEEPING_PATTERN_EXAMPLES][PATTERN_SAMPLE_COUNT][DIMENSION_COUNT] = {
  {
    // PATTERN_NO_MOVEMENT
    {{-36,-65,95},{-32,-58,97},{-24,-81,91},{-43,-65,95},{-28,-64,95},{-31,-71,92}},
    {{-20,-66,92},{-26,-73,81},{-29,-73,98},{-25,-65,96},{-45,-73,93},{-19,-74,96}},
    {{-27,-69,90},{-20,-60,81},{-25,-87,101},{-31,-53,94},{-20,-81,91},{-42,-59,89}},
    {{-20,-83,98},{-23,-58,93},{-35,-63,87},{-28,-75,98},{-35,-65,99},{-43,-66,88}},
    {{-25,-69,96},{-36,-62,87},{-7,-93,95},{-27,-52,102},{-40,-59,85},{-18,-74,86}},
    {{-37,-69,96},{-34,-69,102},{-24,-76,96},{-25,-64,86},{-37,-75,94},{-32,-49,91}},
    {{-28,-71,93},{-27,-71,86},{-18,-60,79},{-18,-52,88},{-21,-68,84},{3,-77,89}},
    {{-33,-56,103},{-18,-101,94},{-29,-44,92},{-27,-70,95},{-33,-61,90},{-20,-67,83}},
    {{-14,-69,99},{-49,-63,90},{-30,-71,91},{-9,-74,83},{-42,-53,81},{-17,-87,104}},
    {{-29,-77,91},{-41,-39,86},{-23,-74,88},{-29,-74,91},{-35,-69,98},{-19,-68,99}}
  },
  {
    // PATTERN_GENTLE_MOVEMENT
    {{-31,-78,98},{-39,-66,99},{-31,-67,98},{-36,-54,102},{-35,-61,92},{-49,-81,103}},
    {{-36,-42,87},{-14,-66,89},{11,-83,93},{-12,-36,91},{32,-70,98},{59,-45,74}},
    {{-62,-66,95},{-82,-69,98},{-57,-87,103},{-83,-49,83},{-116,-40,79},{-83,-74,86}},
    {{60,-24,70},{12,-65,85},{-50,-82,99},{1,-54,94},{28,-73,73},{-60,-45,81}},
    {{-47,-74,101},{-39,-36,90},{-25,-66,83},{-24,-67,82},{-3,-66,97},{-6,-82,101}},
    {{-13,-73,89},{-37,-64,91},{-35,-65,87},{-31,-78,91},{-26,-76,84},{-55,-70,100}},
    {{-17,-79,97},{-32,-63,91},{-16,-64,95},{-6,-65,93},{93,-50,75},{-19,-44,83}},
    {{-37,-78,84},{-55,-65,90},{-78,-61,79},{-100,-112,115},{-69,-81,105},{-40,-51,89}},
    {{18,-61,93},{-14,-57,91},{-16,-48,95},{-5,-20,88},{-5,-66,86},{-61,-76,99}},
    {{-33,-60,93},{-55,-81,93},{-103,-53,89},{-124,-54,98},{-57,-50,87},{-8,-52,91}}
  },
  {
    // PATTERN_HEAD_MOVEMENT
    {{-134,271,203},{529,-82,216},{657,-21,271},{1006,-96,393},{1454,-80,336},{1773,-99,231}},
    {{307,-889,252},{-207,147,-4},{-3756,2520,-324},{-3089,780,39},{-1101,-100,63},{149,-36,68}},
    {{2472,860,-302},{290,31,-8},{-257,-101,111},{-129,139,-66},{-347,-60,135},{259,199,-22}},
    {{140,-92,77},{-60,-13,67},{-51,-173,157},{29,-112,116},{552,-914,286},{761,-1505,311}},
    {{1297,1873,-415},{1985,879,-336},{986,390,-178},{-491,-201,164},{-170,518,-147},{-342,-545,92}},
    {{-3761,2022,-279},{-208,656,-67},{645,-257,88},{212,-79,75},{115,-198,140},{-40,-17,106}},
    {{1618,624,-291},{140,-463,135},{-23,178,53},{-22,127,-12},{206,639,-230},{-108,-73,123}},
    {{131,-372,170},{906,-259,112},{546,-281,147},{294,-323,142},{-134,-8,101},{214,-376,174}},
    {{-429,30,94},{-404,119,87},{156,-91,52},{122,-92,58},{-181,-409,118},{-150,-809,113}},
    {{-53,-101,119},{338,-452,271},{1293,-1499,510},{811,-1368,524},{359,-1594,713},{147,-749,248}}
  },
  {
    // PATTERN_RESTLESS_BODY
    {{-39,87,56},{-10,116,39},{-84,124,36},{-21,56,22},{57,-292,168},{37,-307,177}},
    {{-51,-120,144},{-148,-331,216},{-71,-43,109},{-44,91,29},{77,-158,68},{-80,31,84}},
    {{-71,-169,86},{82,-17,64},{-99,22,99},{-102,-93,83},{139,-232,109},{-7,-45,92}},
    {{97,-216,130},{-150,-33,113},{-2,-169,100},{-4,-57,82},{-18,-91,93},{-35,-44,83}},
    {{-56,-51,85},{95,-207,106},{-61,-22,85},{-82,40,64},{-13,-56,58},{3,-98,92}},
    {{14,18,62},{-31,-105,102},{-12,-49,95},{-17,-40,81},{-47,-12,71},{1,26,61}},
    {{-42,-63,88},{-64,12,52},{-85,46,53},{35,-51,81},{-82,-217,161},{-73,-182,166}},
    {{-38,29,41},{-50,133,1},{128,-129,86},{3,-128,114},{39,6,80},{23,-25,93}},
    {{-116,-76,119},{29,-64,97},{9,-104,97},{3,-269,113},{21,-51,64},{-79,-52,114}},
    {{38,-90,95},{-11,-65,93},{-5,-20,89},{1,-84,62},{52,-91,88},{-65,83,34}}
  },
  {
    // PATTERN_TOSSING_TURNING
    {{2782,1052,-795},{4249,2078,-238},{2553,6062,-934},{-2031,-1619,238},{-1172,-1460,724},{-3110,-4023,795}},
    {{165,193,-85},{-177,2867,-680},{-658,4347,-1146},{-65,-524,259},{-4,377,-56},{-566,3015,-627}},
    {{-326,10872,-2104},{895,175,-270},{895,558,-389},{1260,-1018,-116},{646,-2174,423},{-62,-5055,1508}},
    {{309,-196,382},{-1237,-1159,844},{-3261,-4200,1692},{-3884,-2211,328},{120,-118,74},{-79,-247,137}},
    {{1935,2407,-960},{33,1757,-66},{-628,-1662,319},{-1094,-992,496},{-444,516,209},{-235,1297,-248}},
    {{864,-3242,794},{-56,-338,147},{-173,262,-44},{105,585,-91},{1422,-681,1},{-158,494,688}},
    {{124,-120,113},{-128,-525,225},{323,-689,234},{173,37,-8},{80,707,-9},{-115,1142,-171}},
    {{181,-997,535},{-47,-446,241},{219,142,-100},{1021,2527,-844},{741,2301,-887},{383,3060,-758}},
    {{-201,-569,376},{-1735,2531,-608},{-2050,-96,470},{-503,-1139,469},{714,-2023,425},{535,-1089,197}},
    {{-521,2993,-930},{-401,474,132},{168,-446,173},{68,91,-121},{-888,-647,820},{-3771,1954,-707}}
  }  
};

// Initialize this device as Bluetooth Low Energy device.
// Additionally set characteristcs to share with connected devices.
// More info about Bluetooth LE can be found here:
// https://www.arduino.cc/en/Reference/CurieBLE

BLEPeripheral blePeripheral;  
BLEService slumberService("180F"); 

// READ ONLY "movement detected"
BLEUnsignedCharCharacteristic movementChar("180F", BLERead | BLENotify);


void setup() {

  if(DEBUG_MODE) { 
    Serial.begin(9600); // initialize Serial communication
  }

  // GYROSCOPE INIT 
  CurieIMU.begin();       
  // Set the accelerometer range to 250 degrees/second
  CurieIMU.setGyroRange(250); 

  // attach interrupt for shock (shake) detection.
  CurieIMU.attachInterrupt(checkShake);

  // set shock (shake detection) thresholds.
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 3000);
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50);  // span in ms.
  CurieIMU.interrupts(CURIE_IMU_SHOCK);

  // BLUETOOTH LE INIT
  blePeripheral.setLocalName("SLUMBER");
  blePeripheral.setAdvertisedServiceUuid(slumberService.uuid()); // use slumber uuid
  blePeripheral.addAttribute(slumberService);
  blePeripheral.addAttribute(movementChar); // add movementDetectedChar
  blePeripheral.begin(); // begin BT
  
  // set BT char values (on startup)
  movementChar.setValue(0);
}


/**
 * heartPulse
 * This method makes the LED attached to the heart pulse.
 * Higher numbers cause the heart to pulse faster, lower numbers - slower.
 * :: param - speed (0-100)
 **/
void heartPulse(int speed) {
  
  int maxFadeValue = 255;       // the maximum fade value for analogWrite.
  int fadeIncrement = 5;        // how much to fade in each iteration.
  
  int pulseCount = 8;          // number of times to pulse heart.
  int delayValue = 105-speed;    // how long to wait in between each LED brightness change
  
  for(int pulseValue=0; pulseValue<pulseCount; pulseValue+=1) {
    // run the fade in process
    for (int fadeValue=0; fadeValue<=maxFadeValue; fadeValue+=fadeIncrement) {
      // write to analog for each fade increment, then wait.
      analogWrite(LED_PIN, fadeValue);
      delay(delayValue);
    }
    // turn LED_PIN off.
    analogWrite(LED_PIN,0);    
  }
}

/**
 * signalRecord 
 * This method turns on the heart LED, flips the status of the `recordingActive` global variable, 
 * waits 5 seconds, then turns off the LED.
 **/
void signalRecord() {
  // turn on LED.
  analogWrite(LED_PIN,200); 
  
  // wait a bit for shaking to stop.
  delay(5000);
  
  if(recordingActive) { 
    // flash heart for results of sleep.
    float sleepResult = (float)recordingNoMovements / (float)recordingTotalRecords * 100.0;
    
    // log serial if DEBUG_MODE enabled.
    if(DEBUG_MODE) {
      Serial.println("==Sleep Tracking Complete==");
      Serial.print("Total Records Made: ");
      Serial.println(recordingTotalRecords);
      Serial.print("No Movements Made: ");
      Serial.println(recordingNoMovements);
      Serial.print("Final Sleep Score: ");
      Serial.println(sleepResult);
    }
    
    // activate heart pulse with score.
    heartPulse((int)sleepResult);
    
    // reset all recording activity.
    recordingActive = false; 
    recordingNoMovements = 0;
    recordingTotalRecords = 0;
    
  } else { 
    if(DEBUG_MODE) { Serial.println("==Sleep Tracking Started=="); }
    recordingActive = true; 
  }  
  
  // turn off LED.
  analogWrite(LED_PIN,0); 
}

/**
 * checkShake
 * This method checks the accelerometer to start/stop the recording process.
 **/
void checkShake(void) {
  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
    signalRecord();
  }  
} 


 
 
/**
 * recordSleep
 * This method reads data from gyroscope in 100ms chunks and saves it to global sleep matrix.
 * :: param - void
 * :: return - void (writes result to global `sleepMatrix`)
 **/
void recordSleep() {
  int gX,gY,gZ = 0;
  for(int currSample=0; currSample<PATTERN_SAMPLE_COUNT; currSample+=1) {
  
    CurieIMU.readGyro(gX,gY,gZ);
    sleepMatrix[currSample][0] = gX;
    sleepMatrix[currSample][1] = gY;
    sleepMatrix[currSample][2] = gZ;
    delay(100);
  }

}

/**
 * diagnoseSleepRecordItem
 * This method compares two parts of the sleepMatrix and SLEEP_DATA for matching qualities.
 * :: param - sm1, sleep matrix (first entry)
 * :: param - sm2, sleep matrix (second entry)
 * :: param - sd1, sleep data (first entry)
 * :: param - sd2, sleep data (second entry)
 * :: return - didMatch (bool) - true if pattern matched
 **/
bool diagnoseSleepRecordItem(int sm1, int sm2, int sd1, int sd2) {

  int smDiff = abs(sm1)-abs(sm2);
  int sdDiff = abs(sd1)-abs(sd2);
  
  // check to see if sleep matrix falls within the sleep data range 
  int upperBound = sdDiff+PATTERN_MATCH_THRESHHOLD;
  int lowerBound = sdDiff-PATTERN_MATCH_THRESHHOLD;
 
  if(upperBound>smDiff && lowerBound<smDiff) return true;
  else return false;
  
}

/**
 * diagnoseSleepRecord
 * This method reads the global sleep matrix and attempts to infer sleep patterns from it.
 * :: param - patternType (0-4) // type of sleep pattern to check against.
 * :: param - patternEx (0-9) // which example to compare against.
 * :: param - useSection (0-5) // which section of the sleep record to use.
 * :: return - sleepPattern (int)
 **/
int diagnoseSleepRecord(int patternType, int patternEx, int useSection) {
  
  bool passValues[DIMENSION_COUNT] = { false };
  int nextSection = useSection+1; // index of nextSection

  
  for(int i=0; i<DIMENSION_COUNT; i++) {
    passValues[i] = diagnoseSleepRecordItem(
      sleepMatrix[useSection][i],
      sleepMatrix[nextSection][i],
      SLEEP_DATA[patternType][patternEx][useSection][i],
      SLEEP_DATA[patternType][patternEx][nextSection][i]
    );
  }

  // return 1 if all dimensions feel within the range of the record.
  if(passValues[0] && passValues[1] && passValues[2]) return 1;
  else return 0;
  
}

/**
 * diagnoseSleep
 * This method reads the global sleep matrix and attempts to infer sleep patterns from it.
 * :: param - void
 * :: return - sleepPattern (int)
 **/
int diagnoseSleep() {
  
  // loop through sleep recognition patterns.
  for(int i=0; i<SLEEPING_PATTERNS; i++) {

    for(int j=0; j<SLEEPING_PATTERN_EXAMPLES; j++) {
      // we do 1 less than the PATTERN_SAMPLE_COUNT because we are comparing the differerence
      // between two SLEEPING_PATTERN_EXAMPLES. 
      int matchesMade = 0;

      for(int k=0; k<PATTERN_SAMPLE_COUNT-1;k++) {
        // we can now compare the active sleep matrix, with our sleep pattern samples.
        // ex: {{-36,-65,95},{-32,-58,97},{-24,-81,91},{-43,-65,95},{-28,-64,95},{-31,-71,92}}
        matchesMade += diagnoseSleepRecord(i,j,k);
      }
      
      if(matchesMade >= MATCHES_REQUIRED) {
        // match found - stop here and return this sleep pattern code.
        return i;
      }
      
    }
  }
  
  // unsure about sleep pattern.
  return -1;
  
}


void printSleepMatrix() {
  Serial.print("{"); // outer open bracket.
  for(int i=0; i<PATTERN_SAMPLE_COUNT; i++) {
    Serial.print("{");
    for (int j=0; j<DIMENSION_COUNT; j++) {
      // print each individual sleepMatrix entry.
      Serial.print(sleepMatrix[i][j]);
      // print comma if not last value.
      if(j!=DIMENSION_COUNT-1) Serial.print(",");
    }
    Serial.print("}"); // close bracket.
    // print comma if not last value.
    if(i!=PATTERN_SAMPLE_COUNT-1) Serial.print(",");   
  }
  Serial.print("}\r"); // outer close brack + newline.
  // keep matrix entries separate.
  Serial.print("------------------ \r");
} 


void pillowProcesses() {
  recordSleep();
  printSleepMatrix();
  int sleepResult = diagnoseSleep();
  
  // track pillow movement during recording activity.
  if(recordingActive) {
    // add one to no movement count if sleepResult was 0.
    if(sleepResult==0) { recordingNoMovements++; }
    // add one to the total count of records made.
    recordingTotalRecords++;
  }
  
  // broadcast sleep result if BLE enabled
  movementChar.setValue(sleepResult);
}


void loop() {

  long previousMs = 0;
  BLECentral central = blePeripheral.central();

  Serial.println("Waiting for BLE connection...");
  
  // only process data if central present. 
  if(central) {
    Serial.println("BLE connection established!");  
    while (central.connected()) {
      long currentMs = millis();
      // 1000ms passed? recheck and save sensor data.
      if (currentMs - previousMs >= 1000) {
        previousMs = currentMs;
        // run pillow processes.
        pillowProcesses();
      }
    
    } // end central conenction
  } // end centreal check
} // end loop.

// that's all folks ;)
