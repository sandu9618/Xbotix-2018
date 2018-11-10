//void whiteSqure() {
//  qtrrc.read(sensors);
//  for (int i = 0; i < 8; i++) {
//    if (sensors[i] < 300) {
//      count++;
//    }
//  }

  //----------------Motors----------------------------------------------------------------------------------------------------------------------
  void goAndTurnLeft()
  {
    motorPIDLineFOllow();
    delay(adjGoAndTurn);
    turnLeft(leftSpeed);
  }

  //---------------------------------------------------
  void goAndTurnRight()
  {
    motorPIDLineFOllow();
    delay(adjGoAndTurn);
    turnRight(rightSpeed);
  }
  void Speed(byte leftSpeed, byte rightSpeed)
  {
    analogWrite(leftPWM, leftSpeed);
    analogWrite(rightPWM, rightSpeed);
  }

  void motorStop()
  { digitalWrite(rightMotorForward, LOW);
    digitalWrite(rightMotorBack, LOW);
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBack, LOW);
  }


  void goLeftMotorAt (long leftDistance , int Speed , bool derection ) {
    if (derection == 1) {
      digitalWrite(leftMotorForward, HIGH);
      analogWrite(leftPWM, Speed);

      attachInterrupt(2, leftISR, CHANGE);
      if (leftcount > leftDistance) {
        digitalWrite(leftMotorForward, LOW);
        motorStop();
        analogWrite(leftPWM, 0);
      }

    }
    else if (derection == 0) {
      digitalWrite(leftMotorBack, HIGH);
      analogWrite(leftPWM, Speed);

      attachInterrupt(2, leftISR, CHANGE);
      if (leftcount > leftDistance) {
        digitalWrite(leftMotorBack, LOW);
        motorStop();
        analogWrite(leftPWM, 0);
      }

    }
  }
  // ========================================================================================================================================
  void goRightMotorAt(long rightDistance, int Speed, bool derection) {


    if (derection == 1) {
      digitalWrite(rightMotorForward, HIGH);
      analogWrite(rightPWM, Speed);
      attachInterrupt(3, rightISR, CHANGE);

      if (rightcount > rightDistance) {
        motorStop();
        digitalWrite(rightMotorForward, LOW);
        analogWrite(rightPWM, 0);
      }
    }
    else if (derection == 0) {
      digitalWrite(rightMotorBack, HIGH);
      analogWrite(rightPWM, Speed);
      attachInterrupt(3, rightISR, CHANGE);

      if (rightcount > rightDistance) {
        digitalWrite(rightMotorBack, LOW);
        motorStop();
        analogWrite(rightPWM, 0);
      }
    }
  }

  //---------------------------------------------


  //------------------------------------------------
  void turnLeft(int Speed)
  {
    digitalWrite(rightMotorBack, LOW);
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBack, HIGH);
    digitalWrite(rightMotorForward, HIGH);

    analogWrite(leftPWM, Speed);
    analogWrite(rightPWM, Speed);
  }

  //------------------------------------------------
  void turnRight(int Speed)
  {
    digitalWrite(rightMotorBack, HIGH);
    digitalWrite(leftMotorForward, HIGH);
    digitalWrite(leftMotorBack, LOW);
    digitalWrite(rightMotorForward, LOW);

    analogWrite(leftPWM, Speed);
    analogWrite(rightPWM, Speed);;
  }
  //---------------------------------------------------
  void go3_CM(int Speed)
  {
    goRightMotorAt(rightONEround , Speed, forward );
    goLeftMotorAt(leftONEround / 15, Speed ,  forward);
  }

  //---------------------------------------------------
  void go6_CM(int Speed) {
    goRightMotorAt(rightONEround , Speed, forward );
    goLeftMotorAt(leftONEround , Speed ,  forward);
  }
  //---------------------------------------------------

  //------------------------------------------------

  void turnBack(int Speed) {
    digitalWrite(rightMotorBack, LOW);
    digitalWrite(leftMotorForward, LOW);
    digitalWrite(leftMotorBack, HIGH);
    digitalWrite(rightMotorForward, HIGH);

    analogWrite(leftPWM, Speed);
    analogWrite(rightPWM, Speed);
  }

  //--------------------------------------------------------------------------------------------------------------------------------------------------



  //----------------Position Detection----------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------------------------------------------------------------------------------------------------------


  //----------------Mesh Solve----------------------------------------------------------------------------------------------------------------------

  void mazeSolve(void)
  {
    while (!status)
    {
      //readLFSsensors();
      Serial.print("kf");
      junctionDT();
      switch (mode)
      {
        case NO_LINE:
          breack();
          delay(20);
          backTurn();
          delay(20);
          breack();
          recIntersection('B');
          Serial.print("B");
          delay(20);
          followingLine();
          //sensorJunctionValues();
          break;

        case CONT_LINE:
          breack();
          delay(30);
          forward_bit_9CM(10);
          leftcount = rightcount = 0 ;
          //readLFSsensors();
          junctionDT();
          if (mode != CONT_LINE) {
            leftTurn();
            /*90 degrees*/ recIntersection('L'); Serial.print("L"); // or it is a "T" or "Cross"). In both cases, goes to LEFT
          }
          else mazeEnd();
          break;

        case RIGHT_TURN:
          breack();
          delay(30);
          forward_bit_9CM(50);
          breack();
          Serial.print("went");
          //readLFSsensors();
          junctionDT();
          if (mode == NO_LINE) {
            rightTurn();
            Serial.print("turned");
            /*90 degrees*/ recIntersection('R'); Serial.print("R");
          }
          else {
            followingLine();
            recIntersection('S');
            Serial.print("S");
          }
          break;

        case LEFT_TURN:
          breack();
          forward_bit_9CM(50);
          breack();
          leftTurn(); //90 degrees
          delay(20);
          breack();
          recIntersection('L'); Serial.print("L");
          break;

        case FOLLOWING_LINE:
          followingLine();
          break;

      }
    }
  }
  //------------------------------------------------------------------------
  void recIntersection(char direction)
  {
    path[pathLength] = direction; // Store the intersection in the path variable.
    pathLength ++;
    simplifyPath(); // Simplify the learned path.
  }
  //--------------------------------------------------------------------------

  void mazeEnd(void)
  {
    motorStop();
    for (int i = 0; i < pathLength; i++)
      Serial.print(path[i]);
    Serial.print("  pathLenght ==> ");
    Serial.println(pathLength);
    status = 1;
    mode = STOPPED;
  }
  void gohome(void)
  {
    motorStop();
    for (int i = 0; i < pathLength; i++)
      Serial.print(path[i]);
    Serial.print("  pathLenght ==> ");
    Serial.println(pathLength);
    a = 1;
    mode = STOPPED;
  }
  //--------------------------------------------------------------------------

  void simplifyPath()
  {
    // only simplify the path if the second-to-last turn was a 'B'
    if (pathLength < 3 || path[pathLength - 2] != 'B')
      return;

    int totalAngle = 0;
    int i;
    for (i = 1; i <= 3; i++)
    {
      switch (path[pathLength - i])
      {
        case 'R':
          totalAngle += 90;
          break;
        case 'L':
          totalAngle += 270;
          break;
        case 'B':
          totalAngle += 180;
          break;
      }
    }

    // Get the angle as a number between 0 and 360 degrees.
    totalAngle = totalAngle % 360;

    // Replace all of those turns with a single one.
    switch (totalAngle)
    {
      case 0:
        path[pathLength - 3] = 'S';
        break;
      case 90:
        path[pathLength - 3] = 'R';
        break;
      case 180:
        path[pathLength - 3] = 'B';
        break;
      case 270:
        path[pathLength - 3] = 'L';
        break;
    }

    // The path is now two steps shorter.
    pathLength -= 2;

  }

  //------------------------------------------------------------------------------

  void mazeOptimization (void)
  {
    while (!a)
    {
      //readLFSsensors();
      junctionDT();
      switch (mode)
      {
        case FOLLOWING_LINE:
          followingLine();
          break;
        case CONT_LINE:
          if (pathIndex >= pathLength) gohome ();
          else {
            mazeTurn (path[pathIndex]);
            pathIndex++;
          }
          break;
        case LEFT_TURN:
          if (pathIndex >= pathLength) gohome ();
          else {
            mazeTurn (path[pathIndex]);
            pathIndex++;
          }
          break;
        case RIGHT_TURN:
          if (pathIndex >= pathLength) gohome ();
          else {
            mazeTurn (path[pathIndex]);
            pathIndex++;
          }
          break;
      }
    }
  }
  //---------------------------------------------------------------------------


  void mazeTurn (char dir)
  {
    switch (dir)
    {
      case 'L': // Turn Left
        breack();
        forward_bit_9CM(70);
        breack();
        leftTurn();
        break;

      case 'R': // Turn Right
        breack();
        forward_bit_9CM(70);
        breack();
        junctionDT();
        if (mode == NO_LINE) {
          rightTurn();
          break;

        case 'B': // Turn Back
          breack();
          delay(20);
          forward_bit_9CM(70);
          breack();
          delay(200);
          backTurn();
          recIntersection('B');
          Serial.print("B");
          breack();
          delay(200);
          followingLine();
          break;

        case 'S': // Go Straight
          followingLine();
          break;
        }
    }
  }

    //------------------------------------------------------------------------------------------------------------------------------------------------


    //----------------Selete Rode----------------------------------------------------------------------------------------------------------------------


    //------------------------------------------------------------------------------------------------------------------------------------------------


    //----------------Hole Detection------------------------------------------------------------------------------------------------------------------

    bool detectHole() {
      DownSensor = sonarDown.ping_cm() * 10;
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------


    //----------------Wall Follow---------------------------------------------------------------------------------------------------------------------


    void goInWall() {
      setWallFollow();
      Read_LR_Sonar();
      Find_walls();
      if (leftwall == true && rightwall == true) {
        leftWallFollow();
        Serial.println("both");
      }
      else if (leftwall == true && rightwall == false) {
        leftWallFollow();
        Serial.println("left");
      }
      else if (leftwall == 0 && rightwall == false) {
        rightWallFollow();
        Serial.println("right");
      }
    }


    void setWallFollow()
    {
      lastError = 0;
      integral = 0;

      Kp = 1;
      Kd = 50;
      Ki = 0.01;

      minSpeed = 100;
      baseSpeed = 150;
      maxSpeed = 200;
    }


    //--wall sensor reading--//
    void Read_LR_Sonar() {
      lSensor = sonarLeft.ping_cm() * 10; //ping in cm
      rSensor = sonarRight.ping_cm() * 10;

      leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
      rightSensor = (rSensor + oldRightSensor) / 2;

      oldLeftSensor = leftSensor; // save old readings for movment
      oldRightSensor = rightSensor;

      Serial.print("L : ");
      Serial.print(leftSensor);
      Serial.print("    ");
      Serial.print("R : ");
      Serial.println(rightSensor);

    }

    //--checkk for walles--//
    void Find_walls() {

      if ( rightSensor != 0 ) {

        if ( rightSensor < wall_threshold ) {
          rightwall = true ;
          Serial.print("right");
          Serial.println("  ");
        }
        else {
          rightwall = false ;
        }
      }

      if ( leftSensor != 0 ) {
        if ( leftSensor < wall_threshold ) {
          leftwall = true ;
          Serial.print("left");
          Serial.println("  ");
        }
        else {
          leftwall = false ;
        }
      }


    }


    //============================================================================================
    //=========== left Wall Follow ===============================================================
    void leftWallFollow()
    {
      int error = leftSensor - 50; //mm

      integral = integral + error;

      byte controlSpeed = Kp * error + Kd * (error - lastError) + Ki * integral;

      lastError = error;

      byte rightMotorSpeed = constrain((baseSpeed + controlSpeed), minSpeed, maxSpeed);
      byte leftMotorSpeed = constrain((baseSpeed - controlSpeed), minSpeed, maxSpeed);

      Speed(leftMotorSpeed, rightMotorSpeed);
    }

    //============================================================================================
    //=========== right Wall Follow ===============================================================
    void rightWallFollow()
    {
      int error = rightSensor - 50; //mm

      integral = integral + error;

      byte controlSpeed = Kp * error + Kd * (error - lastError) + Ki * integral;

      lastError = error;

      byte rightMotorSpeed = constrain((baseSpeed - controlSpeed), minSpeed, maxSpeed);
      byte leftMotorSpeed = constrain((baseSpeed + controlSpeed), minSpeed, maxSpeed);

      Speed(leftMotorSpeed, rightMotorSpeed);
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------


    //---------------- ----------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------------------------

    //----------------Coloum counting-----------------------------------------------------------------------------------------------------------------

    void ReadCountSensor() {
      CSensor = sonarCount.ping_cm();                  //ping in cm
      CountSensor = (CSensor + oldCountSensor) / 2;   //average distance between old & new readings to make the change smoother
      oldCountSensor = CountSensor;                   // save old readings for movment
    }
    void ColumCount() {

      for (countPosition = 0; countPosition <= 45; countPosition += 1) // goes from 0 degrees to 45 degrees
      {
        Countservo.write(countPosition);
        delay(50);
        ReadCountSensor();
        rpole = rpole + CountSensor;
        Serial.println(CountSensor);

      }
      delay(50);

      for (countPosition = 45; countPosition <= 135; countPosition += 1) // goes from 15 degrees to 135 degrees
      {
        Countservo.write(countPosition);
        delay(15);
        ReadCountSensor();
        mpole = mpole + CountSensor;
        Serial.println(CountSensor);

      }
      delay(50);

      for (countPosition = 135; countPosition <= 180; countPosition += 1) // goes from 135 degrees to 180 degrees
      {
        Countservo.write(countPosition);
        delay(15);
        ReadCountSensor();
        lpole = lpole + CountSensor;
        Serial.println(CountSensor);

      }

      if (lpole > 100) {
        numOfPole = numOfPole + 1;
      }
      if (rpole >= 100) {
        numOfPole = numOfPole + 1;
      }
      if (mpole >= 100) {
        numOfPole = numOfPole + 1;
      }
      Serial.print("lpole=");
      Serial.println(lpole);
      Serial.print("mpole=");
      Serial.println(mpole);
      Serial.print("rpole=");
      Serial.println(rpole);
      Serial.print("numOfPole=");
      Serial.println(numOfPole);
    }

    void verifyCount() {
      int n = 0;
      int num[3];
      while (n != 3) {
        ReadCountSensor() ;
        ColumCount();
        num[n] = numOfPole;
        n++;
      }
      if (num[0] == num[1] == num[2]) {
        numOfPole = num[0];
      }
      else if (num[0] == num[1]) {
        numOfPole = num[0];
      }
      else if (num[1] == num[2]) {
        numOfPole = num[1];
      } else {
        numOfPole = num[2];
      }

    }

    //------------------------------------------------------------------------------------------------------------------------------------------------

    //------------------------------------------------------------------------------------------------------------------------------------------------


    //----------------Line Follow---------------------------------------------------------------------------------------------------------------------

    void followingLine(void)
    {
      //readLFSsensors();
      calculatePIDLineFollow();
      motorPIDLineFOllow();


      //  int count = 0 ;
      //  for (int i = 0 ; i < 7 ; i++){
      //    if (sensors[i] > 300 ){
      //      count++;
      //      }
      //    }
      //  if(count == 7){
      //    wait();
      //    //breack();
      //    }
    }
    void calculatePIDLineFollow() {

      int position = qtrrc.readLine(sensors, QTR_EMITTERS_ON, 1); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
      int error = position - 3500;
      //Serial.print("Error=");
      //Serial.println(error);
      integral = integral + error;
      motorSpeed = Kp * error + Kd * (error - lastError) + Ki * integral;
      lastError = error;

      for (int i = 0 ; i < 8 ; i++) {
        Serial.print(sensors[i]);
        Serial.print("  ");
      }
      Serial.println("  ");
    }

    void motorPIDLineFOllow() {

      rightMotorSpeed = rightBaseSpeed + motorSpeed;//+
      leftMotorSpeed = leftBaseSpeed - motorSpeed;//-

      if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the right motor from going beyond max speed
      if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the left motor from going beyond max speed
      if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
      if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

      {
        digitalWrite(rightMotorForward, HIGH);
        digitalWrite(rightMotorBack, LOW);
        analogWrite(rightPWM, rightMotorSpeed);

        digitalWrite(leftMotorForward, HIGH);
        digitalWrite(leftMotorBack, LOW);
        analogWrite(leftPWM, leftMotorSpeed);
      }
    }
    //-------------------------------------------------------------------------------------------------------------------------------------------------
    //==================junction===================
    void junctionDT() {

      sensorJunctionValues();
      junction = ' ' ;
      if ( L1 < whiteBLACK  && R1 > whiteBLACK   ) {   // left : turn left
        junction = 'L'; mode  = LEFT_TURN ;
      }

      else if ( L1 > whiteBLACK  && R1 < whiteBLACK  ) {  // right : line end-->turn right, line not end--> go forward
        junction = 'R'; mode = RIGHT_TURN ;
      }

      else if ( L1 < whiteBLACK  && R1 < whiteBLACK   && M < whiteBLACK ) {  // T  : turn left
        junction = 'T'; mode = CONT_LINE ;
      }

      else if ( L1 > whiteBLACK  && R1 > whiteBLACK  && M > whiteBLACK  ) {  // back :
        junction = 'B'; mode = NO_LINE ;
      }
      else {
        mode = FOLLOWING_LINE ;
      }


      Serial.println(junction);
    }

    //====================== Junction value ===============
    void sensorJunctionValues() {
      // read raw sensor values
      qtrrc.read(sensorValues);
      L1 = sensorValues[7];

      R1 = sensorValues[0];

      M  = sensorValues[3];

      int i = 0;
      for (i = 0; i < 7; i++) {
        Serial.print(sensorValues[i]);
        Serial.print( " " );
      }
      Serial.println(" ");
    }

    //------------------------------------------------------------------------------------------------------------------------------------------------
    //-------------------------------/// Encoder function ///-----------------------------

    void wait() {
      analogWrite(rightPWM, 0);
      analogWrite(leftPWM, 0);
    }
    void leftISR() {
      leftcount++;
      // Serial.print(leftcount);
    }
    void rightISR() {
      rightcount++;
    }

    //----------------Color Detection-----------------------------------------------------------------------------------------------------------------
    void ColorFind() {
      // Setting red filtered photodiodes to be read
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      // Reading the output frequency
      frequency = pulseIn(sensorOut, LOW);
      r = frequency;
      //Remaping the value of the frequency to the RGB Model of 0 to 255
      //frequency = map(frequency, 14600, 348, 0, 255);
      // Printing the value on the serial monitor
      Serial.print("R= ");//printing name
      Serial.print(frequency);//printing RED color frequency
      Serial.print("  ");
      delay(1);


      // Setting Green filtered photodiodes to be read
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      // Reading the output frequency
      frequency = pulseIn(sensorOut, LOW);
      g = frequency;
      //Remaping the value of the frequency to the RGB Model of 0 to 255
      //frequency = map(frequency, 147450, 350, 0, 255);
      // Printing the value on the serial monitor
      Serial.print("G= ");//printing name
      Serial.print(frequency);//printing RED color frequency
      Serial.print("  ");
      delay(1);


      // Setting Blue filtered photodiodes to be read
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
      // Reading the output frequency
      frequency = pulseIn(sensorOut, LOW);
      b = frequency;
      //Remaping the value of the frequency to the RGB Model of 0 to 255
      //frequency = map(frequency, 11100, 258, 0, 255);
      // Printing the value on the serial monitor
      Serial.print("B= ");//printing name
      Serial.print(frequency);//printing RED color frequency
      Serial.print("  ");
      delay(1);

      if (r > g && r > b) {
        color = "RED";
        Serial.print(color);
      }
      if (g > r && g > b) {
        color = "GREEN";
        Serial.print(color);
      }
      if (b > g && b > r) {
        color = "BLUE";
        Serial.print(color);
      }

      Serial.println("  ");
    }
    //------------------------------------------------------------------------------------------------------------------------------------------------

    //----------------Robo Arm------------------------------------------------------------------------------------------------------------------------
    //////Loading - First Round /////////////
    ////----------------------------/////////

    //////Unloading - First Round ///////////
    ////----------------------------/////////

    //////Loading - Second Round ////////////
    ////----------------------------/////////

    //////Unloading - Second Round //////////
    ////----------------------------/////////
    //------------------------------------------------------------------------------------------------------------------------------------------------

