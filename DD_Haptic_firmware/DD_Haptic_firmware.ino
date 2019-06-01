#include <DDHapticHelper.h>


//sample code

typedef enum {
  WAIT,
  SAMPLE,
  PLAYBACK,
} States;

static States currentState = WAIT;

char val;
int buttonID;

//sample global variables
float myKp1 = 75;
float myKi1 = 0.75;
float myKd1 = 250; //  * Kp;
bool isSaturated = false;
bool finishSampling = false;
float lastPos = 0;
float lastForce = 0;
float maxForce = 600;
float maxProfileForce = 10000;
float forceIncFill = 200;
float posIncFill = posRes;

//playback global variables
float myKp2 = 3 ;
float myKi2 = 0.0; //0.12;
float myKd2 = 0.0; //0.01 * Kp;

/*Velocity calculation*/
float xh = 0;           // position of the handle [m]
float xh_prev;          // Distance of the handle at previous time step
float xh_prev2;
float dxh;              // Velocity of the handle
float dxh_prev;
float dxh_prev2;
float dxh_filt;         // Filtered velocity of the handle
float dxh_filt_prev;
float dxh_filt_prev2;
long velTime = 10;
long lastVelUpdate = 0;
long currVelUpdate = 0;

float Kd_vel = 0;

/*force filter*/
int thisForce = 0;
#define ffWindowSize 50
float ffWindow[ffWindowSize];
int ffIndex = 0;
float positionVal = 0;

/* SD setup */
#include <SD.h>
#include <SPI.h>
File myPosFile;
File myForceFile;
const int chipSelect = BUILTIN_SDCARD;

/* post processing setup*/
float outputPos[dataSize]; 
float outputForce[dataSize];

float lastPosData = -1;
float currPosData = -1;
float localForceSum = 0;
float localCount = 0;
float maxAngle = 500;
int cutoffIndex = 0;

void setup() {
  Serial.begin(115200);

  /* PID setup */
  Input = 0;
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  int myRes = 12;
  myPID.SetOutputLimits(-pow(2, myRes), pow(2, myRes));
  myPID.SetSampleTime(100);   // Sample time 100 micros
  myPID.SetTunings(myKp1, myKi1, myKd1);
  analogWriteResolution(myRes);
  /* Encoder ABI setup */
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_I, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoderAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoderAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_I), updateEncoderI, RISING);
  /* Motor setup */
  pinMode(pwmPin0, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  analogWriteFrequency(pwmPin0, 18000);
  analogWriteFrequency(pwmPin1, 18000);
  pinMode(enablePin0, OUTPUT);
  digitalWrite(enablePin0, HIGH);
  /* Button setup */
  debouncer.attach(buttonPin, INPUT_PULLUP);
  debouncer.interval(25);
  for (int i = 0; i < filterWindowSize; i++) {
    filterWindow[i] = 0;
  }
  pinMode(LED_BUILTIN, OUTPUT);
  blinkNTimes(5, 200);

  SD.begin(chipSelect);
}

void loop() {
//  sample();

  eventChecker();
  switch (currentState)
  {
    case WAIT:
      analogWrite(pwmPin0, 0);
      analogWrite(pwmPin1, 0);
      if (val == 's') { //sample
        buttonID = Serial.parseInt();
        myPID.SetTunings(myKp1, myKi1, myKd1);
        isSaturated = false;
        finishSampling = false;
        Setpoint = 0;
        currentState = SAMPLE;
      } else if (val == 'p') {
        buttonID = Serial.parseInt();
//        blinkNTimes(3, 200);
        currentState = PLAYBACK;
      }
      break;
    case SAMPLE:
      sample();
      if (val == 'd' || finishSampling) {
        if (finishSampling) Serial.println('d');
        digitalWrite(LED_BUILTIN, LOW);
        postProcess(cutoffIndex);
        currentState = WAIT;
      } 
      else if (val == 'R') Serial.println(positionVal);

      break;
    case PLAYBACK:
      playback();
      if (val == 'd') {
        blinkNTimes(1, 200);
        currentState = WAIT;
      }
      else if (val == 'R') Serial.println(positionVal);
      break;
  }
}


void eventChecker() {
  if (Serial.available()) {
    val = Serial.read();
  } else {
    val = '0';
  }
} 

//playback functions
void playback() {
  myPID.SetTunings(myKp2, myKi2, myKd2);
  
  updateEncoderAB();
  positionVal = filterEncoderAB();
  thisForce = updateRawForce();
  Input = filterForce();
  updateVelocity();

  Setpoint = calculateSetpoint();

  if (myPID.Compute()) {
    Output -= dxh_filt * Kd_vel;
    offsetOutput(800.0, 4096.0);

    pwmVal0 = (abs(Output) + Output) / 2;
    pwmVal1 = (abs(Output) - Output) / 2;
    analogWrite(pwmPin0, pwmVal0);
    analogWrite(pwmPin1, pwmVal1);
  }
}

float filterForce() {
  ffWindow[ffIndex] = thisForce;
  float ffFilt = averageBuf(ffWindow, ffWindowSize);
  ffIndex++;
  ffIndex %= ffWindowSize;
  return ffFilt;
}

void updateVelocity() {
  currVelUpdate = millis();
  if (currVelUpdate - lastVelUpdate > velTime) {
    xh = Input_pos;
    dxh = (double)(xh - xh_prev);
    // Calculate the filtered velocity of the handle using an infinite impulse response filter
    dxh_filt = .85 * dxh + 0.1 * dxh_prev + 0.05 * dxh_prev2;

    xh_prev2 = xh_prev;
    xh_prev = xh;

    dxh_prev2 = dxh_prev;
    dxh_prev = dxh;

    dxh_filt_prev2 = dxh_filt_prev;
    dxh_filt_prev = dxh_filt;

    lastVelUpdate = currVelUpdate;
  }
}

//sample functions
void sample() {
  blinkSample();
  if (!isSaturated) {
    updateEncoderAB();
    positionVal = filterEncoderAB();
    Input = Input_pos;
    int thisForce = updateRawForce();

    if (myPID.Compute()) {
      offsetOutput(800.0, 4096.0);
//      myPID.SetTunings(myKp1, myKp1, myKp1);
      pwmVal0 = (abs(Output) - Output) / 2;
      pwmVal1 = (abs(Output) + Output) / 2;
      analogWrite(pwmPin0, pwmVal0);
      analogWrite(pwmPin1, pwmVal1);
    }
    // Take sample and take into the buffers
    posBuffer[bufCount] = Input;
    forceBuffer[bufCount] = thisForce;
  } else {
    posBuffer[bufCount] = lastPos + posIncFill;
    forceBuffer[bufCount] = lastForce + forceIncFill;
    analogWrite(pwmPin0, 0);
    analogWrite(pwmPin1, 0);
  }
  bufCount++;
  // When buffers are full, take the average of buffer and save into data arrays
  if (bufCount == bufSize) {
    posData[dataCount] = averageBuf(posBuffer, bufSize); //print to processing
    forceData[dataCount] = averageBuf(forceBuffer, bufSize); //print to proc.
    if (forceData[dataCount] >= maxForce && lastForce >= maxForce) {
      isSaturated = true;
      cutoffIndex = dataCount;
    }
    lastPos = posData[dataCount];
    lastForce = forceData[dataCount];
    lastForce = min(lastForce, maxProfileForce);
    dataCount++;
    Setpoint += posRes;
    bufCount %= bufSize;
  }
  // When data arrays are full, print them
  if (dataCount == dataSize) {
    finishSampling = true;
    analogWrite(pwmPin0, 0);
    analogWrite(pwmPin1, 0);
  }
}

const char *posFileNames[] = {"pos1.txt", "pos2.txt", "pos3.txt", "pos4.txt", "pos5.txt"};
const char *forceFileNames[] = {"force1.txt", "force2.txt", "force3.txt", "force4.txt", "force5.txt"};

void saveDataToSD() {
  if (SD.exists(posFileNames[buttonID - 1])) SD.remove(posFileNames[buttonID - 1]);
  myPosFile = SD.open(posFileNames[buttonID - 1], FILE_WRITE);
  if (myPosFile) {
    for (int i = 0; i < dataSize; i++) {
      myPosFile.println(outputPos[i]);
    }
    myPosFile.close();
  }

  if (SD.exists(forceFileNames[buttonID - 1])) SD.remove(forceFileNames[buttonID - 1]);
  myForceFile = SD.open(forceFileNames[buttonID - 1], FILE_WRITE);
  if (myForceFile) {
    for (int i = 0; i < dataSize; i++) {
      myForceFile.println(outputForce[i]);
    }
    myForceFile.close();
  }
}

void postProcess(int cutoffIndex) {
  int j = 0;
  for (int i = 0; i <= cutoffIndex; i++) {
    float currPosData = posData[i];
    if (currPosData != lastPosData && i != 0) {
      outputPos[j] = lastPosData;
      outputForce[j] = localForceSum / localCount;
      localForceSum = 0;
      localCount = 0;
      j++;
    }
    localForceSum += forceData[i];
    localCount += 1;
    lastPosData = currPosData;
  }
  outputPos[j] = lastPosData;
  outputForce[j] = localForceSum / localCount;

  float m = 500;
  float posDiff = outputPos[j] - outputPos[j-1];
  float lastForceData = m * posDiff + outputForce[j];
  for (int k = j + 1; k < dataSize; k++) {
    outputPos[k] = outputPos[k-1] + posDiff; 
    outputForce[k] = lastForceData;
  }
  saveDataToSD();
}



unsigned int lastBlinkTime = 0;
unsigned int currBlinkTime = 0;
unsigned int blinkInterval = 500;
bool blinkState = LOW;
void blinkSample() {
  currBlinkTime = millis();
  if (currBlinkTime - lastBlinkTime > blinkInterval) {
    digitalWrite(LED_BUILTIN, blinkState);
    lastBlinkTime = currBlinkTime;
    blinkState = !blinkState;
  }
}



void blinkNTimes(int n, int dt) {
  for (int i  = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(dt);
    digitalWrite(LED_BUILTIN, LOW);
    delay(dt);
  }
}
