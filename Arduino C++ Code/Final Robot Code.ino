//***** ARDUINO CODE FOR SURVEILLANCE CAMERA ROBOT****//
#include "Servo.h"        //Servo Motor Library
#include "DHT.h"          //DHT Sensors Library
#define DHTTYPE DHT11     //defining the type of dht used in robot
Servo tiltControl;        //Tilt Motion Conrol Object

char receivedChar;
boolean newData = false;
//defining the digital pins of dc geared motors
const int dcMotor1pin1 = 23;
const int dcMotor1pin2 = 25;
const int dcMotor2pin1 = 27;
const int dcMotor2pin2 = 29;
const int dcMotor3pin1 = 31;
const int dcMotor3pin2 = 33;
const int dcMotor4pin1 = 35;                                        
const int dcMotor4pin2 = 37;
//defining the digital pins of elevation motion stepper motor
const int elevationStep = 39;
const int elevationDirt = 41;
//defining the digital pins of circular motion stepper motor
const int circularStep = 43;
const int circularDirt = 45;
//defining the digital pins of the object detection ultrasonic sensors
const int ultraSonic1Trig = 2;
const int ultraSonic1Echo = 3;
const int ultraSonic2Trig = 4;
const int ultraSonic2Echo = 5;
const int ultraSonic3Trig = 6;
const int ultraSonic3Echo = 7;
const int ultraSonic4Trig = 8;
const int ultraSonic4Echo = 9;
//defining the digital pins of environmental sensors
const int dht11pin = 47;
const int mq4pin = A0;
const int mq7pin = A1;
const int mq135pin = A2;
const int Servopin = A0;

int tiltangle = 0;      //tilt motion angle set to zero on start
int elevDist = 0;       //elevation motion distance from bottom set to zero on start
int circDist = 0;       //circualr motion angle set to zero on startup
int dist1object = 0;    //distance of the nearby object to ultrasonic sensor 1
int dist2object = 0;    //distance of the nearby object to ultrasonic sensor 2
int dist3object = 0;    //distance of the nearby object to ultrasonic sensor 3
int dist4object = 0;    //distance of the nearby object to ultrasonic sensor 4

void setup() {                          //initial setup of MCU
  Serial.begin(9600);                   //serial communication begin at baud rate of 9600 bps
  //setting all the motor driving pins to OUTPUT mode and sensors pins to INPUT mode
  pinMode(dcMotor1pin1, OUTPUT);        
  pinMode(dcMotor1pin2, OUTPUT);
  pinMode(dcMotor2pin1, OUTPUT);
  pinMode(dcMotor2pin2, OUTPUT);
  pinMode(dcMotor3pin1, OUTPUT);
  pinMode(dcMotor3pin2, OUTPUT);
  pinMode(dcMotor4pin1, OUTPUT);
  pinMode(dcMotor4pin2, OUTPUT);
  pinMode(elevationStep, OUTPUT);
  pinMode(elevationDirt, OUTPUT);
  pinMode(circularStep, OUTPUT);
  pinMode(circularDirt, OUTPUT);
  pinMode(ultraSonic1Echo, OUTPUT);
  pinMode(ultraSonic1Trig, OUTPUT);
  pinMode(ultraSonic2Echo, OUTPUT);
  pinMode(ultraSonic2Trig, OUTPUT);
  pinMode(ultraSonic3Echo, OUTPUT);
  pinMode(ultraSonic3Trig, OUTPUT);
  pinMode(ultraSonic4Echo, OUTPUT);
  pinMode(ultraSonic4Trig, OUTPUT);

  tiltControl.attach(Servopin);
  tiltControl.write(0);               //making sure servo is at zero angle on startup

  DHT dht(dht11pin, DHTTYPE) ;
  dht.begin()                         //ask dht 11 sensor to start transmission of fata

  pinMode(mq4pin, INPUT);
  pinMode(mq7pin, INPUT);
  pinMode(mq135pin, INPUT);  
}

void loop() {                           //infinte loop of MCU 
  //functions to control differnet aspect of robot
  objectDetection();
  Motion();
  temp_humid();
  EnvSensor();
}

void objectDetection()                  //object detection sensor mechanis
{
  //making all the trigger pins of all sensors LOW simultaneously
  digitalWrite(ultraSonic1Trig, LOW);
  digitalWrite(ultraSonic2Trig, LOW);
  digitalWrite(ultraSonic3Trig, LOW);
  digitalWrite(ultraSonic4Trig, LOW);
  Delay(2);
  //making all the trigger pins of all sensors HIGH simultaneously and then with a 10ms delay make the LOW again
  digitalWrite(ultraSonic1Trig, HIGH);
  digitalWrite(ultraSonic2Trig, HIGH);
  digitalWrite(ultraSonic3Trig, HIGH);
  digitalWrite(ultraSonic4Trig, HIGH);
  Delay(10);
  digitalWrite(ultraSonic1Trig, LOW);
  digitalWrite(ultraSonic2Trig, LOW);
  digitalWrite(ultraSonic3Trig, LOW);
  digitalWrite(ultraSonic4Trig, LOW);
  //time the pulses received from all the Echo pins and convert them into distances (cm)
  dist1object = pulseIn(ultraSonic1Echo, HIGH)*0.034/2;
  dist2object = pulseIn(ultraSonic2Echo, HIGH)*0.034/2;
  dist3object = pulseIn(ultraSonic3Echo, HIGH)*0.034/2;
  dist4object = pulseIn(ultraSonic4Echo, HIGH)*0.034/2;
  //transmits all the data serially to the Rasp Pi 
  Serial.write(dist1object);
  Serial.write(dist2object);
  Serial.write(dist3object);
  Serial.write(dist4object);
}

void Motion()                           //all of the robotic motions function
{
  if (Serial.available() > 0) {         //check to see if user asked any function to perform
    receivedChar = Serial.read();
    newData = true;
  }
  int cmd = (receivedChar - '0');

  while(newData == true) 
  {
    if(cmd == 1)                          //forward chassis motion
    {
      if (dist1object > 10)               //making sure object is 10cm away from Sensor 1
      {
        digitalWrite(dcMotor1pin1, HIGH);
        digitalWrite(dcMotor1pin2, LOW);
        digitalWrite(dcMotor2pin1, HIGH);
        digitalWrite(dcMotor2pin2, LOW);
        digitalWrite(dcMotor3pin1, HIGH);
        digitalWrite(dcMotor3pin2, LOW);
        digitalWrite(dcMotor4pin1, HIGH);
        digitalWrite(dcMotor4pin2, LOW);
        Delay(5000); // 5sec delay  
      }
    }
    else if(cmd == 2)                     //reverse chassis motion
    {
      if (dist2object > 10)               //making sure object is 10cm away from Sensor 2
      {
        digitalWrite(dcMotor1pin1, LOW);
        digitalWrite(dcMotor1pin2, HIGH);
        digitalWrite(dcMotor2pin1, LOW);
        digitalWrite(dcMotor2pin2, HIGH);
        digitalWrite(dcMotor3pin1, LOW);
        digitalWrite(dcMotor3pin2, HIGH);
        digitalWrite(dcMotor4pin1, LOW);
        digitalWrite(dcMotor4pin2, HIGH);
        Delay(5000); // 5sec delay
      }
    }
    else if(cmd == 3)                     //left chassis motion
    {
      if(dist3object > 10)                //making sure object is 10cm away from Sensor 3
      {
        digitalWrite(dcMotor1pin1, LOW);
        digitalWrite(dcMotor1pin2, LOW);
        digitalWrite(dcMotor2pin1, HIGH);
        digitalWrite(dcMotor2pin2, LOW);
        digitalWrite(dcMotor3pin1, LOW);
        digitalWrite(dcMotor3pin2, HIGH);
        digitalWrite(dcMotor4pin1, LOW);
        digitalWrite(dcMotor4pin2, LOW);
        Delay(5000); // 5sec delay
      }
    }
    else if(cmd == 4)                     //right chassis motion
    {
      if(dist4object > 10)                //making sure object is 10cm away from Sensor 4
      {
        digitalWrite(dcMotor1pin1, HIGH);
        digitalWrite(dcMotor1pin2, LOW);
        digitalWrite(dcMotor2pin1, LOW);
        digitalWrite(dcMotor2pin2, LOW);
        digitalWrite(dcMotor3pin1, LOW);
        digitalWrite(dcMotor3pin2, LOW);
        digitalWrite(dcMotor4pin1, LOW);
        digitalWrite(dcMotor4pin2, HIGH);
        Delay(5000); // 5sec delay
      }
    }
    else if(cmd==5)                       //stop all the movements of robot
    {
      digitalWrite(dcMotor1pin1, LOW);
      digitalWrite(dcMotor1pin2, LOW);
      digitalWrite(dcMotor2pin1, LOW);
      digitalWrite(dcMotor2pin2, LOW);
      digitalWrite(dcMotor3pin1, LOW);
      digitalWrite(dcMotor3pin2, LOW);
      digitalWrite(dcMotor4pin1, LOW);
      digitalWrite(dcMotor4pin2, LOW);
      digitalWrite(elevationDirt, LOW);
      digitalWrite(elevationStep, LOW);
      digitalWrite(circularStep, LOW);
      digitalWrite(circularDirt, LOW);
    }
    else if (cmd ==6)                      //elevation vertically up
    {
      if (elevDist != 300)                 //making sure the Al block hasnt reached the top
      { 
        digitalWrite(elevationDirt, HIGH);  //stepper motor direction so as to make camera move up
        for (int x =0; x< 1000; x++)        //1000 pulses (= 1cm up dist)
        {
          digitalWrite(elevationStep, HIGH);
          delayMicroseconds(800);
          digitalWrite(elevationStep, LOW);
          delayMicroseconds(800);
        }
        elevDist++;                          //vertical distance couter inc 
      }
    }
    else if (cmd ==7)                        //elevation vertically down
    {
      if (elevDist != 0)                    //making sure the Al block hasnt reached the bottom
      {
        digitalWrite(elevationDirt, LOW);   //stepper motor direction so as to make camera move down
        for (int x =0; x<1000; x++)
        {
          digitalWrite(elevationStep, HIGH);
          delayMicroseconds(800);
          digitalWrite(elevationStep, LOW);
          delayMicroseconds(800);
        }
        elevDist--;
      }
    }
    else if (cmd ==8)                       //circular motion clockwise
    {
      if (circDist ! = 720)                 //making sure robot dont make 2 rot in +ve dir
      {
        digitalWrite(circularDir, HIGH);     //stepper motor clockwise direction 
        for (int x =0; x < 5; x++)           //5 pulse (=9 degrees)
        {
          digitalWrite(circularStep, HIGH);
          delayMicroseconds(800);
          digitalWrite(circularStep, LOW);
          delayMicroseconds(800);
        }
        circDist = circDist + 5;              //circular angle counter
      } 
    }
    else if (cmd ==9)                         //circular motion anticlockwise
    {
      if(circDist != -720)                    //making sure robot dont make 2 rot in -ve dir
      {
        digitalWrite(circularDir, LOW);       //stepper motor counter clockwise direction 
        for (int x =0; x < 5; x++)
        {
          digitalWrite(circularStep, HIGH);
          delayMicroseconds(800);
          digitalWrite(circularStep, LOW);
          delayMicroseconds(800);
        }
        circDist = circDist - 5;
      }
    }
    else if (cmd == 10)                       //servo motion positive
    {
      if (tiltangle != 90)                    //if angle hasnt crossed the 90 degree limit
      {
        for (int angleT = tiltangle; angleT < tiltangle + 15; angleT++) //15 degree +ve turn
        {
          tiltControl.write(angleT);
          delay(15);
        }
        tiltangle = tiltangle + 15;           //tilt counter
      }
    }
    else if (cmd ==11)                        //servo motion negative
    {
      if (tiltangle != -90)                   //if angle hasnt crossed the -90 degree limit
      {
        for (int angleT = tiltangle; angleT > tiltangle - 15; angleT--) //15 degree -ve turn
        {
          tiltControl.write(angleT);
          delay(15);
        }
        tiltangle = tiltangle - 15;         
      }
    }
    else     //if no user input is detected, make sure no movement is done
    {
      digitalWrite(dcMotor1pin1, LOW);
      digitalWrite(dcMotor1pin2, LOW);
      digitalWrite(dcMotor2pin2, LOW);
      digitalWrite(dcMotor2pin1, LOW);
      digitalWrite(dcMotor3pin1, LOW);
      digitalWrite(dcMotor3pin2, LOW);
      digitalWrite(dcMotor4pin1, LOW);
      digitalWrite(dcMotor4pin2, LOW);
      digitalWrite(elevationDirt, LOW);
      digitalWrite(elevationStep, LOW);
      digitalWrite(circularDirt, LOW);
      digitalWrite(circularStep, LOW); 
    }
      newData = false;  
  }
}

void temp_humid()                       //tep & humidity sensor function
{
  float humidity = dht.readHumidity() ;  //reading humidity value from dht11
  float temp = dht.readTemperature() ;   //reading temp value from dht11
  serial.write(humidity);
  serial.write(temp);
}

void EnvSensor()                        //Environmental sensor function
{
  int methaneVal = analogRead(mq4pin);    //reading analogue val from MQ4 sensor
  int COval= analogRead(mq7pin);          //reading analogue val from MQ7 sensor
  int CO2Val = analogRead(mq135pin);      //reading analogue val from MQ135 sensor
  //trasnmitting these values to raspberry pi
  serial.write(methaneVal);
  serial.write(COval);
  serial.write(CO2val);
}