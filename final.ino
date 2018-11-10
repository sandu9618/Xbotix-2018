#include <QTRSensors.h> // QTR Sensor
#include <NewPing.h>    //sonar sensor
#include <Servo.h>      //Servo motor
#include "define.h"


void setup() {
  //-----------------------Encoder--------------
  pinMode(20 , INPUT_PULLUP);
  pinMode(21 , INPUT_PULLUP);

  interrupts();
  attachInterrupt(2, leftISR, CHANGE);
  attachInterrupt (3, rightISR, CHANGE);

  int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
    qtrrc.calibrate();
  delay(20);
  wait();
  delay(2000); // wait for 2s to position the bot before entering the main loop



  //-----------motors--------------------------------------------------------------------------------------------------------------------------------
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBack, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBack, OUTPUT);

  pinMode(rightPWM, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  //------------------------------------------------------------------------------------------------------------------------------------------------


  //-----------Color--------------------------------------------------------------------------------------------------------------------------------
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  //-----------------------------------------------------------------------------------------------------------------------------------------------



  //----------Counting--------------------------------------------------------------------------------------------------------------------------------
  Countservo.attach(7);  // attaches the Counting servo on pin 7 to the servo object
  //---------------------------------------------------------------------------------------------------------------------------------------------------




  //----------wall follow-----------------------------------------------------------------------------------------------------------------------------

  //---------------------------------------------------------------------------------------------------------------------------------------------------

  //Serial.begin(9600);  // set the data rate in bits per second for serial data transmission
}


void loop() {


  mazeSolve();
  Serial.print("dfg")
;//  verifyCount();
//  backTurn();
//  breack();
//  delay(200);
//  mazeOptimization();
//  breack();
//
//  digitalWrite(leftMotorForward, HIGH);
//  digitalWrite(rightMotorForward, HIGH);
//  analogWrite(rightPWM, 100);
//  analogWrite(leftPWM, 100);
//  delay(1000);
//  breack();
//  //followingLine();
//  junctionDT();
//  while (!(L1 < whiteBLACK)) {
//    followingLine();
//    junctionDT();
//  } breack();
//  leftTurn();
//  breack();
//  followingLine();
//  delay(500);
//  breack();
//  backTurn();
//  junctionDT();
//  while (junction != 'T') {
//    followingLine();
//    junctionDT();
//  } breack();
//  leftTurn();
//  breack();
//  junctionDT();
//  while (junction != 'L') {
//    followingLine();
//    junctionDT();
//  } breack();
//  leftTurn();
//  breack();
//  for (int i = 0; i < 3; i++) {
//    junctionDT();
//    while (junction != 'R') {
//      followingLine();
//      junctionDT();
//    } breack();
//    if (i + 1 == numOfPole) {
//      rightTurn();
//    } else {
//      followingLine();
//    }
//  }
//  junctionDT();
//  while (junction != 'R' || junction != 'T' || junction != 'L') {
//    followingLine();
//    junctionDT();
//  } breack();
//  if (junction == 'L') {
//    leftTurn();
//    junctionDT();
//    while (junction != 'T') {
//      breack();
//      junctionDT();
//      rightTurn();
//      breack();
//    }
//    junctionDT();
//    while (junction != 'T') {
//      followingLine();
//      junctionDT();
//
//    }
//    if (junction == 'R') {
//      rightTurn();
//      junctionDT();
//      while (junction != 'T') {
//        breack();
//        junctionDT();
//        leftTurn();
//        breack();
//      }
//      junctionDT();
//      while (junction != 'T') {
//        followingLine();
//        junctionDT();
//      }
//      if (junction == 'T') {
//        digitalWrite(leftMotorForward, HIGH);
//        digitalWrite(rightMotorForward, HIGH);
//        analogWrite(rightPWM, 80);
//        analogWrite(leftPWM, 80);
//        delay(1000);
//        breack();
//        junctionDT();
//        while (junction != 'T') {
//          followingLine();
//          junctionDT();
//        }
//      }
//       sensorJunctionValues();
//       while(!(junction=='T') ){
//        goInWall();
//        sensorJunctionValues();
//       }
//       breack();
//
//    }
//  }
}

    //===================================================================================================================================
    void leftTurn() {
      //forward_bit_9CM();
      leftcount = 0;
      while (leftcount < 80) {
        digitalWrite(leftMotorBack, HIGH);
        digitalWrite(rightMotorForward, HIGH);
        digitalWrite(rightMotorBack, LOW);
        digitalWrite(leftMotorForward, LOW);

        analogWrite(leftPWM, 100);
        analogWrite(rightPWM, 100);
      }
      Serial.println(leftcount);
      delay(340);
      breack();
      delay(300);
      leftcount = 0;
      breack();
    }


    //===================================================================================================================================
    void rightTurn() {
      //forward_bit_9CM();
      rightcount = 0;
      while (rightcount < 160) {
        digitalWrite(leftMotorBack, LOW);
        digitalWrite(rightMotorForward, LOW);
        digitalWrite(rightMotorBack, HIGH);
        digitalWrite(leftMotorForward, HIGH);
        analogWrite(leftPWM, 100);
        analogWrite(rightPWM, 100);
      }
      Serial.println(rightcount);
      delay(300);
      breack();
      delay(300);
      rightcount = 0;
      breack();
    }


    //===================================================================================================================================
    void backTurn() {
      //forward_bit_9CM();
      leftcount = 0;
      while (leftcount < 480) {
        digitalWrite(leftMotorBack, HIGH);
        digitalWrite(rightMotorForward, HIGH);
        digitalWrite(rightMotorBack, LOW);
        digitalWrite(leftMotorForward, LOW);

        analogWrite(leftPWM, 100);
        analogWrite(rightPWM, 100);
      }
      Serial.println(leftcount);
      delay(30);
      breack();
      delay(30);
      leftcount = 0;
      breack();
    }


    //===============================================================================================================================

    void breack() {

      digitalWrite(leftMotorBack, HIGH);
      digitalWrite(rightMotorForward, HIGH);
      digitalWrite(rightMotorBack, HIGH);
      digitalWrite(leftMotorForward, HIGH);

      //  digitalWrite(leftMotorBack, HIGH);
      //  digitalWrite(rightMotorBack, HIGH);
      //  analogWrite(leftPWM, 200);
      //  analogWrite(rightPWM, 200);
      //  delay(4);
      //  digitalWrite(leftMotorBack, LOW);
      //  digitalWrite(rightMotorForward, LOW);
      //  digitalWrite(rightMotorBack, LOW);
      //  digitalWrite(leftMotorForward, LOW);
      analogWrite(leftPWM, 255);
      analogWrite(rightPWM, 255);
    }

    void forward_bit_9CM(int t)
    {
      digitalWrite(leftMotorBack, LOW);
      digitalWrite(rightMotorBack, LOW);
      digitalWrite(leftMotorForward, HIGH);
      digitalWrite(rightMotorForward, HIGH);

      analogWrite(leftPWM, 200);
      analogWrite(rightPWM, 200);

      delay(t);
      breack();
      delay(100);

    }

    //====================================================================================================================================

























































    void Tleft() {
      leftcount = rightcount = 0 ;
      while (leftcount < 140) {
        turnLeft(220);
      }
      motorStop();
      delay(1000000000);
    }


    void Tright() {
      leftcount = rightcount = 0 ;
      while (rightcount < 250) {
        turnRight(200);
      }
      motorStop();
      //delay(1000000000);
    }

    void Tback() {
      leftcount = rightcount = 0 ;
      while (leftcount < 520) {
        turnLeft(200);
      }
      motorStop();
      // delay(1000000000);
    }

    // ========================================================================================================================================
    void TurnLeft(int Speed) {
      goRightMotorAt(rightONEround * 0.33975 , Speed, forward );
      goLeftMotorAt(leftONEround * 0.343975, Speed ,  backward);
    }



