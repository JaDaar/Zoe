/*****************************************************************************************************************************************
*  PROGRAM:    ZOE003
*  DEVELOPER:  Jeff Hamlin
*  DATE:       08.11.2013
*  DESCRIPTION:
*
  Control program for ZOE BOT.  Zoe has the following IO:
  1.) 2 HC-SR04 Ultrasonic Sensors mounted on Breadboards in the front and rear.
  2.) 1.8" TFT LCD with MicroSD
  3.) Piezo Element for audio feedback
  4.) BT2S Bluetooth Slave
  
  The following code is Zoe's logic to enable autonomous roaming and obstacle avoidance.
  When there are no obstacles she will play the SEEK tone, like she is looking for obstacles and when one is within range of either
  HC-SR04 sensor the reverse direction action be performed with an associated tone.
*  
*****************************************************************************************************************************************/
#include <SoftwareSerial.h>
#include <Servo.h>
#include <TFT.h>
#include <SPI.h>


#define TRIG_FRONT A0
#define ECHO_FRONT A1

#define TRIG_REAR A2
#define ECHO_REAR A3

#define soundPin 3

#define cs   10
#define dc   9
#define rst  8 


#define MOTOR_LEFT 5
#define MOTOR_RIGHT 6
#define MIN_DISTANCE 15

#define BT_RX_PIN 16
#define BT_TX_PIN 17

SoftwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);
TFT TFTscreen = TFT(cs, dc, rst);
int iLine;
Servo motorRight;
Servo motorLeft;

int triggDuration;
int lastTurn;
bool isStopped;
void setup()
{
  //Serial.begin(9600);
  Serial.begin(9600);
  isStopped=false;
  pinMode(soundPin, OUTPUT);
  
  //  Initialize the HC-SR04 sensor pin states
  pinMode(TRIG_FRONT,OUTPUT);
  pinMode(ECHO_FRONT,LOW);
  pinMode(TRIG_REAR,OUTPUT);
  pinMode(ECHO_REAR,LOW);
  
  TFTscreen.begin();
  Serial.begin(9600);
  WriteHeader();
  iLine=10;
  
  //  Setup Futaba S148 Continuous Rotation Servos
  motorLeft.write(1550);  // Rotate to 90
  motorRight.write(1550);
  motorLeft.attach(MOTOR_LEFT);
  motorRight.attach(MOTOR_RIGHT);
  stopRobot();
  forward();
  lastTurn=1;
}

void loop()
{
  float sensingFrontDistance=updateForwardUltrasonicSensor();
  float sensingRearDistance=updateRearUltrasonicSensor();
  WriteMsg("FRONT SENSOR OBJECT DISTANCE: " +   doubleToString(sensingFrontDistance,2),6);
  //Serial.println(sensingFrontDistance);
  Serial.print("REAR SENSOR OBJECT DISTANCE: ");
  Serial.println(sensingRearDistance);

  /******************************************************************************************************************************************
  *
    Zoe's Semi-Autonomous Roaming and Obstacle Avoidance Logic using Two Ultrasonic Sensors on the Front and Rear of the Bot

      PROBLEM: Local Minimum Trap State -     
      // When Zoe executes the turn after the change in direction/delay the sometimes he doesn't negoatiate corners well.
      // he still can get stuck in a situation where the turn will place the object that was in front into the rear causing another turn
      // when the first turn was into another object.  I need to figure out how to have him handle this local minimum trap state.
      //  http://users.ece.gatech.edu/~magnus/Papers/ObAvoidIFAC05.pdf
      //  ftp://web.eecs.umich.edu/people/johannb/paper10.pdf
      //  http://teamcore.usc.edu/people/sorianom/ICRA08.pdf
      //  http://faculty.iiit.ac.in/~mkrishna/jrs1000.pdf
  *
  ******************************************************************************************************************************************/
  if(sensingFrontDistance<=MIN_DISTANCE)
  {
    SetSound(1);
    reverse();  
    turnRobot("FRONT OBJJECT"); 
    delay(1000);  //  May be a dangerous delay since the bot isn't monitoring it's environment
  }
  //else if(sensingRearDistance<=MIN_DISTANCE)
  //{
  //  SetSound(1);
  //  forward();  
  //  turnRobot("REAR OBJECT");   
  //  delay(1000);  //  May be a dangerous delay since the bot isn't monitoring it's environment  
  //}
  else
  {
     // If nothing in front go forward
    if(sensingFrontDistance>MIN_DISTANCE)
      forward(); 
  }
}

String intToString(int input)
{
    //String string = String((int)(input));
    return String((int)input);

}

String doubleToString(double input,int decimalPlaces)
{
  if(decimalPlaces!=0)
  {
    String string = String((int)(input*pow(10,decimalPlaces)));
    if(abs(input)<1)
    {
      if(input>0)
        string = "0"+string;
      else if(input<0)
        string = string.substring(0,1)+"0"+string.substring(1);
    }
    return string.substring(0,string.length()-decimalPlaces)+"."+string.substring(string.length()-decimalPlaces);
  }
  else {
    return String((int)input);
  }
}
void turnRobot(String msg)
{
    if(lastTurn==1)
    {
      turnRight();
      lastTurn=2;
      isStopped=false;
      WriteMsg("TURN LOGIC" + msg + " - COMMENCING RIGHT TURN",5);
    }
    else if(lastTurn==2)      
    {
      turnLeft();
      lastTurn=1;
      isStopped=false;      
      WriteMsg("TURN LOGIC" + msg + " - COMMENCING LEFT TURN",5);
    } //Serial.println("LEFT MOTOR ANGGLE: "+ intToString(motorLeftAngle));
}
float updateRearUltrasonicSensor()
{
  //  Updates the distance of any objects in rear of the bot
  float sensingDistance;
  digitalWrite(TRIG_REAR,HIGH);  
  delayMicroseconds(1000);
  digitalWrite(TRIG_REAR,LOW);
  triggDuration=pulseIn(ECHO_REAR,HIGH);
  sensingDistance=(triggDuration/2)/29.1; 
  return sensingDistance;
}

float updateForwardUltrasonicSensor()
{
  //  Updates the distance of any objects in front of the bot
  float sensingDistance;
  digitalWrite(TRIG_FRONT,HIGH);  
  delayMicroseconds(1000);
  digitalWrite(TRIG_FRONT,LOW);
  triggDuration=pulseIn(ECHO_FRONT,HIGH);
  sensingDistance=(triggDuration/2)/29.1; 
  return sensingDistance;
}
void forward()
{
   reattach();
   WriteMsg("BOT: FORWARD",6);
   SetSound(0);
   motorLeft.write(180);
   motorRight.write(0);
}

void reverse()
{
   WriteMsg("BOT: REVERSE",6);
   motorLeft.write(0);
   motorRight.write(180);
}

void turnRight()
{
   WriteMsg("BOT: RIGHT TURN",5);
   motorLeft.write(200);
   motorRight.write(200);
}

void turnLeft()
{
   WriteMsg("BOT: LEFT TURN",5);
   motorLeft.write(20);
   motorRight.write(20);
}

void stopRobot()
{
   WriteMsg("BOT: STOPPED",4);
   int motorRightAngle=motorRight.read();
   WriteMsg("RIGHT MOTOR ANGGLE: "+ intToString(motorRightAngle),5);
   int motorLeftAngle=motorLeft.read();
   WriteMsg("LEFT MOTOR ANGGLE: "+ intToString(motorLeftAngle),5);
   motorLeft.detach();
   motorRight.detach();
}


void reattach()
{
   if(!motorLeft.attached())
     motorLeft.attach(MOTOR_LEFT);
   if(!motorRight.attached())
     motorRight.attach(MOTOR_RIGHT);
}

void WriteHeader()
{
  TFTscreen.background(0, 0, 0);
  TFTscreen.stroke(255,255,255);
  TFTscreen.setTextSize(2);
  TFTscreen.text("ZOE MONITOR",4,0); 
  TFTscreen.setTextSize(1);
}

void WriteMsg(String robotMessage, int color)
{
   char botMsg[20];
   // convert the reading to a char array
   robotMessage.toCharArray(botMsg, 20);  
   TFTscreen.setTextSize(1);   
   Serial.println(robotMessage); 
   if(iLine==10)
   {
     iLine=20;
     //TFTscreen.stroke(0,250,0); //green
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==20)
   {
     iLine=30;
     //TFTscreen.stroke(0,250,0);
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==30)
   {
     iLine=40;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==40)
   {
     iLine=50;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==50)
   {
     iLine=60;
     //TFTscreen.stroke(235,250,20); //teal
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==60)
   {
     iLine=70;
     //TFTscreen.stroke(235,250,20); //teal
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==70)
   {
     iLine=80;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==80)
   {
     iLine=90;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==90)
   {
     iLine=100;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==100)
   {
     iLine=110;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }
   else if(iLine==110)
   {
     iLine=120;
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
   }   
   else if(iLine==120)
   {
     iLine=20;
     Serial.println("*****************RESET**************");
     SetColor(color);
     TFTscreen.text(botMsg, 0, iLine);
     WriteHeader();
   }
}

void SetColor(int color)
{
   if(color==0)
     TFTscreen.stroke(0,0,0); //  Black Text on Black Bacground
   else if(color==1)
     TFTscreen.stroke(255,255,255);  // WHITE
   else if(color==2)
     TFTscreen.stroke(0,153,255);  // AMBER
   else if(color==3)
     TFTscreen.stroke(225,194,216); // PURPLE
   else if(color==4)
     TFTscreen.stroke(0,0,255); // RED
   else if(color==5)
     TFTscreen.stroke(235,250,20); // TEAL
   else if(color==6)
     TFTscreen.stroke(0,255,0); // GREEN
}

void SetSound(int soundValue)
{
   if(soundValue==0)
     tone(soundPin,3023,100);  //Seek
   else if(soundValue==1)
   {
      //  Reversing
      tone(soundPin,2023,500);
      delay(200);
      tone(soundPin,1562,700);
      delay(400);
   }
    
  
}
