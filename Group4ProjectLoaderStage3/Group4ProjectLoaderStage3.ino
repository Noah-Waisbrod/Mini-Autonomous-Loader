/********************************************************
  MREN 103 - Group4ProjectLoaderStage1

  Original by H. Fernando, 23/09/2021
  Edited by Noah Waisbrod and Mustafa Hasan, 31/03/2022
  
  This code reads sensor data from the minibot and then
  outputs motor speeds to make the bot follow the path
  outlined in the lab.
*********************************************************/
//lib
#include <Servo.h> 
Servo leftWheel;
Servo rightWheel;
Servo myServoA;  // Makes a servo object to control servo A
Servo myServoB;  // Makes a servo object to control servo B

// Pin Assignments
const int RED = 10;          //red LED Pin
const int GRN = 9;           //green LED Pin
const int YLW = 5;           //yellow LED Pin
const int BUTTON = 7;        //pushbutton Pin
int MOTOR_L = 3;            // left motor signal pin
int MOTOR_R = 4;            // right motor signal pin
int SHARP = A3;     // Sharp Sensor on Analog Pin 3

const int LSENSOR = A1; // Left Sensor on Analog Pin 1
const int RSENSOR = A2; // Right Sensor on Analog Pin 2
const boolean PLOT = true;  //true=plot sensor reading; false=serial monitor output.

//global variables
int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value
const int stopPulse = 148;  //resting pulse value (STOP)
const int delta = 8;  //Speed above or below stop (Max Speed)
int loffset = 0;  //offset to slow left wheel
int roffset = 0;  //offset to slow right wheel
int CV = 7; //value to adjust offsets
bool dir = true;

int value = 0;
int mv_value = 0;

int timer =0;
bool start = false;

int servoPinA = 11;     // Bucket servomotor #1 pin
int myAngleA1 = 100;    // initial angle, bucket lifts off ground if too high
int posA = myAngleA1;   // if set to 180, bucket lifts robot off of ground
int myAngleA2 = 152;     // highest angle (lift), puts almost straight, set to 110
                        //    still bent (i.e. not as high)
int servoPinB = 12;     // Bucket servomotor #1 pin
int myAngleB1 = 73;    // initial angle, bucket lifts off ground if too high //70
int posB = myAngleB1;   // if set to 180, bucket lifts robot off of ground
int myAngleB2 = 115;     // highest angle (lift), puts almost straight, set to 110

// Set-up Routine
void setup() {
                
// Initialize led pins as outputs.
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);
  
// Initialize button pins as inputs
  pinMode(BUTTON, INPUT);

// Initialize line following sensor pins as inputs
  pinMode(LSENSOR, INPUT);
  pinMode(RSENSOR, INPUT);

//init sharp
  pinMode(SHARP, INPUT);

// setup motor control pins
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

  runMotors(0,0); // stop the motors to start

//setup arm servos
  myServoA.write(posA);         // Servo A starting position
  myServoA.attach(servoPinA);   // Attaches the servo to the servo object
  myServoB.write(posB);         // Servo A starting position
  myServoB.attach(servoPinB);   // Attaches the servo to the servo object
  
// Initialize serial and monitor
  Serial.begin(9600);     
  digitalWrite(YLW, HIGH);
  digitalWrite(YLW, LOW);
  Serial.println(" "); /// line feed
  Serial.println("Program ready.");
}

long curr;
long tim;

// Main Routine
void loop() {
      //start program by flashing green untill button is pressed 
      do{
           digitalWrite(GRN, HIGH);
           delay(125);
           digitalWrite(GRN, LOW);
           delay(125);
        }while(digitalRead(BUTTON) == LOW);

      //loops program
      while(1){

        curr = millis();
      //read the line sensor value
      lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

      //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,3000);
      rvalue = map(rvalue,0,1023,0,3000);

      //get the distances and convert to milivolts
      value = analogRead(SHARP);
      mv_value = map(value,0,1023,0,3300); //convert AtoD count to millivolts

      //if too far left, slow right wheel
      if(lvalue <= 1000 && rvalue >= 1000){
          turnOnLED(YLW);
          loffset = 0;
          roffset = -CV;
        }
        //if too far right, slow left wheel
        else if(lvalue >= 1000 && rvalue <= 1000){
          turnOnLED(RED);
          loffset = -CV;
          roffset = 0; 
        }
        //if bot is on line, run wheels at same speed
        else if(lvalue <= 1000 && rvalue <= 1000){
          turnOnLED(GRN);
          loffset = 0;
          roffset = 0;
        }
        //if at interection, turn right
        else if(lvalue >= 1000 && rvalue >= 1000 && curr-tim > 8000){
          tim = millis();
          //stop
          turnOnLED(GRN);
          runMotors(0, 0);
          delay(2000);
          //back up for a sec
          runMotors(-2*delta, -2*delta);
          delay(200);
          //turn right
          runMotors(-1*delta,delta);
          delay(1200);
          runMotors(0, 0);
        }
        //if sharp sees bot get too close
        if (mv_value >= 1450){
          turnAround();
          if(dir){
            for (posA = myAngleA1; posA <= myAngleA2; posA++) { // Lift action
              myServoA.write(posA);
              delay(20);
             }
            runMotors(-2*delta+2,-2*delta);
            delay(950);
            runMotors(0, 0);
            for (posA = myAngleA2; posA >= myAngleA1; posA--) { // Lift action
              myServoA.write(posA);
              delay(20);
              }
          runMotors(delta-1,delta+1);
          delay(900);
          dir = false;
          } else {
              runMotors(-2*delta+2,-2*delta);
              delay(1000);
              runMotors(0, 0);
              
              delay(1000);
              for (posB = myAngleB1; posB <= myAngleB2; posB++) {  // Drop action
                myServoB.write(posB);
                delay(20);
              }
            
              //B Down
              delay(1000);
              for (posB = myAngleB2; posB >= myAngleB1; posB--) {  // Drop action
                myServoB.write(posB);
                delay(20);
              }

              
              runMotors(delta-1,delta+1);
              delay(900);
              dir = true;
          
          }
          }
          //runs motors with offsets declared above
        runMotors(delta+loffset,delta+roffset);
      }
}
//********** Functions (subroutines) ******************

// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}

// run robot wheels
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL)*10;    //length of pulse in microseconds
  int pulseR = (stopPulse + deltaR)*10; 
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR); 
}

void turnAround(){
          //stop
          runMotors(0, 0);
          delay(2000);
          //back up
          runMotors(-2*delta+2,-2*delta);
          delay(600);
          runMotors(0,0);
          delay(1000);
          runMotors(delta, -2*delta);
          delay(1200);
          runMotors(0, 0);
     } 
