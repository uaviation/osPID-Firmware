//#define USE_SIMULATION

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "PID_v1_local.h"
#include "EEPROMAnything.h"
#include "PID_AutoTune_v0_local.h"
#include "io.h"

// ***** PIN ASSIGNMENTS *****

const byte buzzerPin = 6;
const byte systemLEDPin = 4;
int switchPin = A0;

const byte EEPROM_ID = 2; //used to automatically trigger and eeprom reset after firmware update (if necessary)

const byte TYPE_NAV=0;
const byte TYPE_VAL=1;
const byte TYPE_OPT=2;

byte mMain[] = {
  0,1,2,3};
byte mDash[] = {
  4,5,6,7};
byte mConfig[] = {
  8,9,10,11};
byte *mMenu[] = {
  mMain, mDash, mConfig};

byte curMenu=0, mIndex=0, mDrawIndex=0;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
//AnalogButton button(A3, 0, 253, 454, 657);
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

unsigned long now, lcdTime, buttonTime,ioTime, serialTime;
boolean sendInfo=true, sendDash=true, sendTune=true, sendInputConfig=true, sendOutputConfig=true;

bool editing=false;

bool tuning = false;

double setpoint=250,input=250,output=50, pidInput=250;

double kp = 2, ki = 0.5, kd = 2;
byte ctrlDirection = 0;
byte modeIndex = 0;
byte highlightedIndex=0;

PID myPID(&pidInput, &output, &setpoint,kp,ki,kd, DIRECT);

double aTuneStep = 20, aTuneNoise = 1;
unsigned int aTuneLookBack = 10;
byte ATuneModeRemember = 0;
PID_ATune aTune(&pidInput, &output);


byte curProfStep=0;
byte curType=0;
float curVal=0;
float helperVal=0;
unsigned long helperTime=0;
boolean helperflag=false;
unsigned long curTime=0;


/*Profile declarations*/
const unsigned long profReceiveTimeout = 10000;
unsigned long profReceiveStart=0;
boolean receivingProfile=false;
const int nProfSteps = 15;
char profname[] = {
  'N','o',' ','P','r','o','f'};
byte proftypes[nProfSteps];
unsigned long proftimes[nProfSteps];
float profvals[nProfSteps];
boolean runningProfile = false;


//for devlopment and demo purposes, it's useful to have a
//simulation that can run on the osPID.  the problem is
//that is uses memory.  rather than have it hogging resources
//when not in use, it's activated using a compile flag.
// this way, it doesn't get compiled during normal  circumstances

#ifdef USE_SIMULATION
double kpmodel = 5, taup = 50, theta[30];
const double outputStart = 50;
const double inputStart=250;

void DoModel()
{
  // Cycle the dead time
  for(byte i=0;i<30;i++)
  {
    theta[i] = theta[i+1];
  }
  // Compute the input
  input = (kpmodel / taup) *(theta[0]-outputStart) + (input-inputStart)*(1-1/taup)+inputStart + ((float)random(-10,10))/100;
}
#else

#endif /*USE_SIMULATION*/



void setup()
{
  Serial.begin(9600);
  lcdTime=10;
  buttonTime=1;
  ioTime=5;
  serialTime=6;
  //windowStartTime=2;
  lcd.begin(8, 2);

  lcd.setCursor(0,0);
  lcd.print(F(" osPID   "));
  lcd.setCursor(0,1);
  lcd.print(F(" Arduino "));
  delay(1000);
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);
  pinMode(switchPin, INPUT);

  initializeEEPROM();



#ifdef USE_SIMULATION
  input = inputStart;
  for(int i=0;i<30;i++)theta[i] = outputStart;
#else
  InitializeInputCard();
  InitializeOutputCard();
#endif
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, 100);
  myPID.SetTunings(kp, ki, kd);
  myPID.SetControllerDirection(ctrlDirection);
  myPID.SetMode(modeIndex);
}

byte editDepth=0;
void loop()
{
  now = millis();

//  if(now >= buttonTime) {
    int reading = digitalRead(switchPin);
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    } 
    if ((millis() - lastDebounceTime) > debounceDelay) {
      buttonState = reading;
    }
    if (reading == LOW){
      if(!runningProfile) StartProfile();
      else StopProfile();
    }
    lastButtonState = reading;
//    buttonTime += 50;  
//  }

  bool doIO = now >= ioTime;
  //read in the input
  if(doIO)
  { 
    ioTime+=250;
#ifdef USE_SIMULATION
    DoModel();
    pidInput = input;
#else
    input =  ReadInputFromCard();
    if(!isnan(input))pidInput = input;

#endif /*USE_SIMULATION*/
  }
  

  if(tuning)
  {
    byte val = (aTune.Runtime());

    if(val != 0)
    {
      tuning = false;
    }

    if(!tuning)
    { 
      // We're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
      EEPROMBackupTunings();
    }
  }
  else
  {
    if(runningProfile) ProfileRunTime();
    //allow the pid to compute if necessary
    myPID.Compute();
  }





  if(doIO)
  {
    //send the output
#ifdef USE_SIMULATION
    theta[29] = output;
#else
    //send to output card
    WriteToOutputCard(output);
#endif /*USE_SIMULATION*/  



  }

  if(now>lcdTime)
  {
    drawLCD();
    lcdTime+=250; 
  }
  if(millis() > serialTime)
  {
    //if(receivingProfile && (now-profReceiveStart)>profReceiveTimeout) receivingProfile = false;
    SerialReceive();
    SerialSend();
    serialTime += 500;
  }
}

void drawLCD()
{
     lcd.setCursor(0, 0);
     // lcd.setCursor(0,row);
      if(runningProfile) {
        lcd.print("+");
        lcd.print(profname);
      } else {
        lcd.print("-");
        lcd.print(profname);
      }
     lcd.setCursor(0, 1); 
      // Print current temperature
      lcd.print(input);
      // Print degree Celsius symbol
      lcd.print((char)223);
      lcd.print("C ");
}

void changeAutoTune()
{
  if(!tuning)
  {
    //initiate autotune
    AutoTuneHelper(true);
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{

  if(start)
  {
    ATuneModeRemember = myPID.GetMode();
    myPID.SetMode(MANUAL);
  }
  else
  {
    modeIndex = ATuneModeRemember;
    myPID.SetMode(modeIndex);
  } 
}





void StartProfile()
{
  if(!runningProfile)
  {
    //initialize profle
    curProfStep=0;
    runningProfile = true;
    calcNextProf();
  }
}
void StopProfile()
{
  if(runningProfile)
  {
    curProfStep=nProfSteps;
    calcNextProf(); //runningProfile will be set to false in here
    setpoint = 0;
  } 
}


void ProfileRunTime()
{
  if(tuning || !runningProfile) return;
  


  boolean gotonext = false;

  //what are we doing?
  if(curType==1) //ramp
  {
    //determine the value of the setpoint
    if(now>helperTime)
    {
      setpoint = curVal;
      gotonext=true;
    }
    else
    {
      setpoint = (curVal-helperVal)*(1-(float)(helperTime-now)/(float)(curTime))+helperVal; 
    }
  }
  else if (curType==2) //wait
  {
    float err = input-setpoint;
    if(helperflag) //we're just looking for a cross
    {

      if(err==0 || (err>0 && helperVal<0) || (err<0 && helperVal>0)) gotonext=true;
      else helperVal = err;
    }
    else //value needs to be within the band for the perscribed time
    {
      if (abs(err)>curVal) helperTime=now; //reset the clock
      else if( (now-helperTime)>=curTime) gotonext=true; //we held for long enough
    }

  }
  else if(curType==3) //step
  {

    if((now-helperTime)>curTime)gotonext=true;
  }
  else if(curType==127) //buzz
  {
    if(now<helperTime)digitalWrite(buzzerPin,HIGH);
    else 
    {
       digitalWrite(buzzerPin,LOW);
       gotonext=true;
    }
  }
  else
  { //unrecognized type, kill the profile
    curProfStep=nProfSteps;
    gotonext=true;
  }





  if(gotonext)
  {
    curProfStep++;
    calcNextProf();
  }
}

void calcNextProf()
{
  if(curProfStep>=nProfSteps) 
  {
    curType=0;
    helperTime =0;
  }
  else
  { 
    curType = proftypes[curProfStep];
    curVal = profvals[curProfStep];
    curTime = proftimes[curProfStep];

  }
  if(curType==1) //ramp
  {
    helperTime = curTime + now; //at what time the ramp will end
    helperVal = setpoint;
  }   
  else if(curType==2) //wait
  {
    helperflag = (curVal==0);
    if(helperflag) helperVal= input-setpoint;
    else helperTime=now; 
  }
  else if(curType==3) //step
  {
    setpoint = curVal;
    helperTime = now;
  }
  else if(curType==127) //buzzer
  {
    helperTime = now + curTime;    
  }
  else
  {
    curType=0;
  }



  if(curType==0) //end
  { //we're done 
    runningProfile=false;
    curProfStep=0;
    Serial.println("P_DN");
    digitalWrite(buzzerPin,LOW);
  } 
  else
  {
    Serial.print("P_STP ");
    Serial.print(int(curProfStep));
    Serial.print(" ");
    Serial.print(int(curType));
    Serial.print(" ");
    Serial.print((curVal));
    Serial.print(" ");
    Serial.println((curTime));
  }

}





const int eepromTuningOffset = 1; //13 bytes
const int eepromDashOffset = 14; //9 bytes
const int eepromATuneOffset = 23; //12 bytes
const int eepromProfileOffset = 35; //136 bytes
const int eepromInputOffset = 172; //? bytes (depends on the card)
const int eepromOutputOffset = 300; //? bytes (depends on the card)


void initializeEEPROM()
{
  //read in eeprom values
  byte firstTime = EEPROM.read(0);
  if(firstTime!=EEPROM_ID)
  {//the only time this won't be 1 is the first time the program is run after a reset or firmware update
    //clear the EEPROM and initialize with default values
    for(int i=1;i<1024;i++) EEPROM.write(i,0);
    EEPROMBackupTunings();
    EEPROMBackupDash();
    EEPROMBackupATune();
    EEPROMBackupInputParams(eepromInputOffset);
    EEPROMBackupOutputParams(eepromOutputOffset);
    EEPROMBackupProfile();
    EEPROM.write(0,EEPROM_ID); //so that first time will never be true again (future firmware updates notwithstanding)
  }
  else
  {
    EEPROMRestoreTunings();
    EEPROMRestoreDash();
    EEPROMRestoreATune();
    EEPROMRestoreInputParams(eepromInputOffset);
    EEPROMRestoreOutputParams(eepromOutputOffset);
    EEPROMRestoreProfile();    
  }
}  



void EEPROMreset()
{
  EEPROM.write(0,0);
}


void EEPROMBackupTunings()
{
  EEPROM.write(eepromTuningOffset,ctrlDirection);
  EEPROM_writeAnything(eepromTuningOffset+1,kp);
  EEPROM_writeAnything(eepromTuningOffset+5,ki);
  EEPROM_writeAnything(eepromTuningOffset+9,kd);
}

void EEPROMRestoreTunings()
{
  ctrlDirection = EEPROM.read(eepromTuningOffset);
  EEPROM_readAnything(eepromTuningOffset+1,kp);
  EEPROM_readAnything(eepromTuningOffset+5,ki);
  EEPROM_readAnything(eepromTuningOffset+9,kd);
}

void EEPROMBackupDash()
{
  EEPROM.write(eepromDashOffset, (byte)myPID.GetMode());
  EEPROM_writeAnything(eepromDashOffset+1,setpoint);
  EEPROM_writeAnything(eepromDashOffset+5,output);
}

void EEPROMRestoreDash()
{
  modeIndex = EEPROM.read(eepromDashOffset);
  EEPROM_readAnything(eepromDashOffset+1,setpoint);
  EEPROM_readAnything(eepromDashOffset+5,output);
}

void EEPROMBackupATune()
{
  EEPROM_writeAnything(eepromATuneOffset,aTuneStep);
  EEPROM_writeAnything(eepromATuneOffset+4,aTuneNoise);
  EEPROM_writeAnything(eepromATuneOffset+8,aTuneLookBack);
}

void EEPROMRestoreATune()
{
  EEPROM_readAnything(eepromATuneOffset,aTuneStep);
  EEPROM_readAnything(eepromATuneOffset+4,aTuneNoise);
  EEPROM_readAnything(eepromATuneOffset+8,aTuneLookBack);
}

void EEPROMBackupProfile()
{
  EEPROM_writeAnything(eepromProfileOffset, profname);
  EEPROM_writeAnything(eepromProfileOffset + 8, proftypes);
  EEPROM_writeAnything(eepromProfileOffset + 24, profvals);
  EEPROM_writeAnything(eepromProfileOffset + 85, proftimes); //there might be a slight issue here (/1000?)
}

void EEPROMRestoreProfile()
{
  EEPROM_readAnything(eepromProfileOffset, profname);
  EEPROM_readAnything(eepromProfileOffset + 8, proftypes);
  EEPROM_readAnything(eepromProfileOffset + 24, profvals);
  EEPROM_readAnything(eepromProfileOffset + 85, proftimes); //there might be a slight issue here (/1000?)
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/

boolean ackDash = false, ackTune = false;
union {                // This Data structure lets
  byte asBytes[32];    // us take the byte array
  float asFloat[8];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array

// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats
void SerialReceive()
{

  // read the bytes sent from Processing
  byte index=0;
  byte identifier=0;
  byte b1=255,b2=255;
  boolean boolhelp=false;

  while(Serial.available())
  {
    byte val = Serial.read();
    if(index==0){ 
      identifier = val;
      Serial.println(int(val));
    }
    else 
    {
      switch(identifier)
      {
      case 0: //information request 
        if(index==1) b1=val; //which info type
        else if(index==2)boolhelp = (val==1); //on or off
        break;
      case 1: //dasboard
      case 2: //tunings
      case 3: //autotune
        if(index==1) b1 = val;
        else if(index<14)foo.asBytes[index-2] = val; 
        break;
      case 4: //EEPROM reset
        if(index==1) b1 = val; 
        break;
      case 5: //input configuration
        if (index==1)InputSerialReceiveStart();
        InputSerialReceiveDuring(val, index);
        break;
      case 6: //output configuration
        if (index==1)OutputSerialReceiveStart();
        OutputSerialReceiveDuring(val, index);
        break;
      case 7:  //receiving profile
        if(index==1) b1=val;
        else if(b1>=nProfSteps) profname[index-2] = char(val); 
        else if(index==2) proftypes[b1] = val;
        else foo.asBytes[index-3] = val;

        break;
      case 8: //profile command
        if(index==1) b2=val;
        break;
      default:
        break;
      }
    }
    index++;
  }

  //we've received the information, time to act
  switch(identifier)
  {
  case 0: //information request
    switch(b1)
    {
    case 0:
      sendInfo = true; 
      sendInputConfig=true;
      sendOutputConfig=true;
      break;
    case 1: 
      sendDash = boolhelp;
      break;
    case 2: 
      sendTune = boolhelp;
      break;
    case 3: 
      sendInputConfig = boolhelp;
      break;
    case 4: 
      sendOutputConfig = boolhelp;
      break;
    default: 
      break;
    }
    break;
  case 1: //dashboard
    if(index==14  && b1<2)
    {
      setpoint=double(foo.asFloat[0]);
      //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
      //   value of "Input"  in most cases (as 
      //   in this one) this is not needed.
      if(b1==0)                       // * only change the output if we are in 
      {                                     //   manual mode.  otherwise we'll get an
        output=double(foo.asFloat[2]);      //   output blip, then the controller will 
      }                                     //   overwrite.

      if(b1==0) myPID.SetMode(MANUAL);// * set the controller mode
      else myPID.SetMode(AUTOMATIC);             //
      EEPROMBackupDash();
      ackDash=true;
    }
    break;
  case 2: //Tune
    if(index==14 && (b1<=1))
    {
      // * read in and set the controller tunings
      kp = double(foo.asFloat[0]);           //
      ki = double(foo.asFloat[1]);           //
      kd = double(foo.asFloat[2]);           //
      ctrlDirection = b1;
      myPID.SetTunings(kp, ki, kd);            //    
      if(b1==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
      else myPID.SetControllerDirection(REVERSE);          //
      EEPROMBackupTunings();
      ackTune = true;
    }
    break;
  case 3: //ATune
    if(index==14 && (b1<=1))
    {

      aTuneStep = foo.asFloat[0];
      aTuneNoise = foo.asFloat[1];    
      aTuneLookBack = (unsigned int)foo.asFloat[2];
      if((!tuning && b1==1)||(tuning && b1==0))
      { //toggle autotune state
        changeAutoTune();
      }
      EEPROMBackupATune();
      ackTune = true;   
    }
    break;
  case 4: //EEPROM reset
    if(index==2 && b1<2) EEPROM.write(0,0); //eeprom will re-write on next restart
    break;
  case 5: //input configuration
    InputSerialReceiveAfter(eepromInputOffset);
    sendInputConfig=true;
    break;
  case 6: //ouput configuration
    OutputSerialReceiveAfter(eepromOutputOffset);
    sendOutputConfig=true;
    break;
  case 7: //receiving profile

    if((index==11 || (b1>=nProfSteps && index==9) ))
    {
      if(!receivingProfile && b1!=0)
      { //there was a timeout issue.  reset this transfer
        receivingProfile=false;
        Serial.println("ProfError");
        EEPROMRestoreProfile();
      }
      else if(receivingProfile || b1==0)
      {
        if(runningProfile)
        { //stop the current profile execution
          StopProfile();
        }
          
        if(b1==0)
        {
          receivingProfile = true;
          profReceiveStart = millis();
        }

        if(b1>=nProfSteps)
        { //getting the name is the last step
          receivingProfile=false; //last profile step
          Serial.print("ProfDone ");
          Serial.println(profname);
          EEPROMBackupProfile();
          Serial.println("Archived");
        }
        else
        {
          profvals[b1] = foo.asFloat[0];
          proftimes[b1] = (unsigned long)(foo.asFloat[1] * 1000);
          Serial.print("ProfAck ");
          Serial.print(b1);           
          Serial.print(" ");
          Serial.print(proftypes[b1]);           
          Serial.print(" ");
          Serial.print(profvals[b1]);           
          Serial.print(" ");
          Serial.println(proftimes[b1]);           
        }
      }
    }
    break;
  case 8:
    if(index==2 && b2<2)
    {
      if(b2==1) StartProfile();
      else StopProfile();

    }
    break;
  default: 
    break;
  }
}


// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  if(sendInfo)
  {//just send out the stock identifier
    Serial.print("\nosPID v1.50");
    InputSerialID();
    OutputSerialID();
    Serial.println("");
    sendInfo = false; //only need to send this info once per request
  }
  if(sendDash)
  {
    Serial.print("DASH ");
    Serial.print(setpoint); 
    Serial.print(" ");
    if(isnan(input)) Serial.print("Error");
    else Serial.print(input); 
    Serial.print(" ");
    Serial.print(output); 
    Serial.print(" ");
    Serial.print(myPID.GetMode());
    Serial.print(" ");
    Serial.println(ackDash?1:0);
    if(ackDash)ackDash=false;
  }
  if(sendTune)
  {
    Serial.print("TUNE ");
    Serial.print(myPID.GetKp()); 
    Serial.print(" ");
    Serial.print(myPID.GetKi()); 
    Serial.print(" ");
    Serial.print(myPID.GetKd()); 
    Serial.print(" ");
    Serial.print(myPID.GetDirection()); 
    Serial.print(" ");
    Serial.print(tuning?1:0);
    Serial.print(" ");
    Serial.print(aTuneStep); 
    Serial.print(" ");
    Serial.print(aTuneNoise); 
    Serial.print(" ");
    Serial.print(aTuneLookBack); 
    Serial.print(" ");
    Serial.println(ackTune?1:0);
    if(ackTune)ackTune=false;
  }
  if(sendInputConfig)
  {
    Serial.print("IPT ");
    InputSerialSend();
    sendInputConfig=false;
  }
  if(sendOutputConfig)
  {
    Serial.print("OPT ");
    OutputSerialSend();
    sendOutputConfig=false;
  }
  if(runningProfile)
  {
    Serial.print("PROF ");
    Serial.print(int(curProfStep));
    Serial.print(" ");
    Serial.print(int(curType));
    Serial.print(" ");
switch(curType)
{
  case 1: //ramp
    Serial.println((helperTime-now)); //time remaining
     
  break;
  case 2: //wait
    Serial.print(abs(input-setpoint));
    Serial.print(" ");
    Serial.println(curVal==0? -1 : float(now-helperTime));
  break;  
  case 3: //step
    Serial.println(curTime-(now-helperTime));
  break;
  default: 
  break;
  
}

  }
  
}









