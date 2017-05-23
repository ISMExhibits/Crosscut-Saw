#include <math.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <FastLED.h>
#include <CmdMessenger.h>
#include <Bounce.h>
#define M 40
#define TIME 1
#define R 2
#define LIMIT 5
#define NUM_LEDS 122
#define RATE 1000
#define FADEIN 30
#define DATA_PIN 11
#define CLOCK_PIN 13

Encoder enc(2,3);
IntervalTimer speedSample;
CmdMessenger cmdMessenger = CmdMessenger(Serial1);
Bounce limitSwitch = Bounce(LIMIT,5);

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint,4,10,0, REVERSE);
CRGB leds[NUM_LEDS];

// This is the list of recognized commands.  
// In order to receive, attach a callback function to these events
enum
{
  kCommandList         , // Command to request list of available commands
  kSetSawVolume        , // Command to set the global volume for saw sounds  
  kSetRingVolume       , // Command to set the global volume for ringing sounds  
  kUpdateSaw           , // Command to update saw position, speed and direction
  kIsRinging           , // Command to switch to ringing sound
  kNewStroke           , // Command to start a new sound
  kStatus              , // Command to request led status
  kTest                , // Play a sound with current settings and show status
  kTestAll             , // Play all the sounds
  kPlayFile            , // Play a particular file
};

// Callbacks define on which received commands we take action

//Colors
uint8_t indicatorHue = 170; 
uint8_t indicatorBrightness = 200; 
uint8_t metronomeHue = 5; 
uint8_t metronomeBrightness = 200;
uint8_t successHue = 95; 
uint8_t successBrightness = 200;

bool debugStream = false;
int brake = 9;  //brake pin
int16_t p = 5500;        //current position
int16_t p_min = 1075;    //absolute minimum position
int16_t p_max = 10480; //absolute maximum position
int16_t v;            //velocity in tick's per second
int16_t v_limit = 2200;
int16_t v_max = 2425;
long lastEnc = 0; //last position sample 
volatile int theta[(2*M)]; //array of position changes
volatile int indx = 0; //index to wrap array
int16_t s_j = 50; // parameter related to relative speed accuracy
int16_t j = 0; // counter related to speed function, could be made local
int8_t sawDir = 0;
int8_t lastSawDir = 0;
int8_t sawDirChange = 0;
elapsedMillis sinceUpdate;
elapsedMillis cycleTime = 0;
elapsedMillis sinceFrame = 0;
elapsedMillis dwell = 0;
elapsedMillis stroke = 0;
int counter = 0;
int iDir = 0;
int iLast = 0;

double pidVel;
double lastPidVel;

int totalError = 0;
int frameCount = 0;
int difficulty = 15;

void setup() {
  Serial1.begin(115200);
  //Pin Assignments
  pinMode(brake,OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7,HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH);
  pinMode(LIMIT,INPUT_PULLUP);
  analogWriteFrequency(brake, 23437.5);
  analogWriteResolution(11);
//Encoder Setup
  enc.write(5500);
  speedSample.begin(sample, 1000);
  s_j = 200/R;
//
Setpoint = 500;
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(0, 2048);
myPID.SetSampleTime(2);

FastLED.addLeds<DOTSTAR, DATA_PIN, CLOCK_PIN, RGB,DATA_RATE_MHZ(12)>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
FastLED.setBrightness(100);
cmdMessenger.printLfCr (true);
}

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(230); } }

int metronome(){  //use 
  if (cycleTime < 1201){
    //uint16_t angle = map(cycleTime,0,1200,0,65535);
    //int16_t r = sin16(angle);
    //return map(r,-32767,32767,0,NUM_LEDS); 
    return (int)((NUM_LEDS/2)*cos(6.28*cycleTime/1200.0)+(NUM_LEDS/2));
  }
  else {
    cycleTime = 0;
    return NUM_LEDS;
  }
}

int colorBounce(){
  int j = constrain(metronome(),0,NUM_LEDS);
  return j;
}

int indicator(long encoder){ //input is encoder position
  int j = constrain(map((int)encoder,p_min,p_max,NUM_LEDS,0),0,NUM_LEDS);
  return j;
}

void newCycle(int i){
    float m = (float)indicator(enc.read());
    float temp = (1200*acos((m-(NUM_LEDS/2))/(NUM_LEDS/2)))/6.28;
    cycleTime = constrain((int)temp,0,600);
    if(i<0){
      cycleTime = 1200-cycleTime;
    }
}

void newStroke(){
  static int d1 = 0;
  static int d2 = 1;
  static int d3 = 0;
  int k;
  int i = indicator(enc.read());
  int d = (int)i-(int)iLast;
  if (d<-1){
    d=-1;
  }
  if (d>1){
    d=1;
  }
  if (d1!=d){
    if(d!=0){
      if (d1==0){
        d3=d2;
        d2=d1;
        d1=d;
      }
      else{
        d3=d2;
        d2=d1;
        d1=0;
       }
    }
    else{
      d3=d2;
      d2=d1;
      d1=0;
    }
    k = d1+d2+d3;
    if(k==0){
      if((totalError/frameCount)<difficulty){
        cmdMessenger.sendCmd(kIsRinging,true);    
      }
      else{
        cmdMessenger.sendCmd(kIsRinging,false); 
      }
      cmdMessenger.sendCmd(kNewStroke);
    }
    if ((abs(k)>1)&&(dwell>400)){
      cmdMessenger.sendCmd(kIsRinging,false);
      cmdMessenger.sendCmd(kNewStroke);
    }
  if(stroke > 300){
  frameCount = 1;
  totalError = 0;
  stroke = 0;
  }
  }
  
//  if (stroke > 300){ //Crude low-pass filter
//    stroke = 0;
//    cmdMessenger.sendCmd(kIsRinging,(bool)r);
//    cmdMessenger.sendCmd(kNewStroke);
//    
//  }
}

void frame(){
  //iLast = indicator(enc.read());
  if (sinceFrame > 2){
  sinceFrame = 0;
  newStroke();
  int i = indicator(enc.read());
  if (iLast != i){
    if (dwell > 2400){
      newCycle(iLast-i);
    }
    dwell = 0;
    iLast = i; 
  }
  int n; 
  int frameError;
 
  if(dwell < 2400){
    n = colorBounce();
    frameError = abs(n-i);
    totalError += frameError;
    frameCount++;
    volUpdate();
    if (frameError<9){
      leds[i] = CHSV(successHue, 255, successBrightness);
      leds[n] = CHSV(successHue, 255, successBrightness);
    }
    else{
      leds[i] = CHSV(indicatorHue, 255, indicatorBrightness);
      leds[n] = CHSV(metronomeHue, 255, metronomeBrightness);
    }
  }
  FastLED.show();
  fadeall();
  }
}

void volUpdate(){
  static long lastSent = 0;
  if(enc.read() != lastSent){
  cmdMessenger.sendCmdStart(kUpdateSaw);
  cmdMessenger.sendCmdArg((int16_t)enc.read());
  cmdMessenger.sendCmdArg((int16_t)(Input+700));
  cmdMessenger.sendCmdEnd();
  lastSent = enc.read();
  }
}

void loop() {
  //debugPlot();
  UpdatePID();
  frame();
  updateLimit();
}

void updateLimit(){
  limitSwitch.update();
  if(limitSwitch.fallingEdge()){
    int a = enc.read();
    Serial.println(a);
    a -= 5500;
    a = abs(a);
    if (a>1000){
      enc.write(5500);
    }
  }
}

void UpdatePID(){
  Input = (float)abs(encSpeed());
  Setpoint = newSetPoint();
  myPID.Compute();
  if(abs(Input-Setpoint)<0.02){
    Output = 0;
  }
  if((sawDirection()==1) && (lastEnc > (long)p_max)){
    Output = 2000;
  }
  if((sawDirection()==2) && (lastEnc < (long)p_min)){
    Output = 2000;
  }
  analogWrite(brake,(int)Output);
  }

float newSetPoint(){
pidVel = posToVel();
if (pidVel < 0){
  pidVel = 0;
}
return (float)pidVel;
}

double posToVel(){
  double pos = (double)enc.read();
  switch(sawDirection()){
    case 0: //saw is stopped
      break;
    case 1:
      if(pos<5500){
        pos = 5500; 
      }
      break;
    case 2:
      if(pos>5500){
        pos = 5500;
      }
      break;    
  }
  double _min = (double)p_min;
  double _max = (double)p_max;
  double v_lim = (double)v_max;
  return v_lim*(sin((float)(6.28*((pos-_min)/(2*(_max-_min))))));
}

void sample(){  //for speed calculation, samples are stored in a ring buffer
  int thisEnc = (int)enc.read();
  theta[indx] = (int)(thisEnc - lastEnc);
  lastEnc = thisEnc;
  //indx = ((indx +1) % (2*M));
  indx += 1;
  if (indx > ((2*M)-1))
  indx = 0;
}

int encSpeed(){
  //noInterrupts();
  int lastest = indx;
  //interrupts();
  int tmp = 0;
  int r = 0;  //result to return
  int d_r = 0; //absolute delta r
  j = 0;  //number of time samples (-1)
  while (j < M){
    int _read = ((lastest + (2*M - j))%(2*M));
   // noInterrupts();
    tmp = *(theta + _read);
   // interrupts();
    r += tmp;
    d_r += abs(tmp);
    if (d_r >= s_j){
      j++;
      //return ((float)r/(j*TIME));
      return (r*100/(j));
    }
    j++;
  }
  j++;
  //return ((float)r/(j*TIME));
  return (r*100/(j*TIME));
//return 1;
}

int sawDirection(){
  long thisEnc = enc.read();
  if (lastEnc==thisEnc){
    sawDir = 0;
    return 0; //stopped
  }
  if (lastEnc<thisEnc){
    sawDir = 1;
    return 1; //increasing
  }
  else{
    sawDir = 2;
    return 2; //decreasing
  }
}

void debugPlot(){
  if ((sinceUpdate > 20)&&(sawDirection() > 0)&&(debugStream==true)){
    Serial.print((enc.read()/2));
    Serial.print(", ");
    Serial.print(abs(encSpeed()));
   // Serial.print(", ");
   // Serial.print(sawDirection());
   // Serial.print(", ");
   // Serial.print(posToVel());
    Serial.print(", ");
    Serial.print(Input);
    Serial.print(", ");
    Serial.print(Setpoint);
    Serial.print(", ");
    Serial.println(Output);
    sinceUpdate = sinceUpdate = 0;
  }
}

