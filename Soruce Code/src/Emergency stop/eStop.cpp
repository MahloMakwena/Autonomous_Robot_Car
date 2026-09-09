void loop() {
      // Check if the hardware interrupt flag was tripped
  if (estopTriggered) 
  {
    // 1. Instantly write software failsafes (e.g., turn off software PWM signals)
        analogWrite(motorPin1, 0); 
        analogWrite(motorPin2, 0); 
    // 2. Lock the software loop permanently until an engineer resets the board
    while (true) 
    {
      Serial.println("!!! SOFTWARE ALERT: HARDWARE E-STOP DETECTED !!!");
      delay(1000); 
    }
  }

  // Your normal autonomous path-tracking or navigation code runs here
  Serial.println("Autonomous vehicle in race.....");
  delay(500); 
  // Move motor forward
  moveForward();
  // Stop
  stop();
  // Move motor backward
   //backward();
  // Stop
  stop();
}

//helper functions for robot motion
void backward()// allows robot to move backwards
{
  Serial.println("Moving backwards....");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  ledcWrite(enablePin, dutyCycle);
  delay(2000);
}
void stop()// stop the robot
{
  Serial.println("Motor Stopped");
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  delay(1000);
}
void moveForward()// allows robot to move forward
{
  Serial.println("Moving forward...");
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  ledcWrite(enablePin, dutyCycle);
  delay(2000);
}
bool isPath(float right,float forward)//function that checks which direction to take
{
    /*get distance from both sides using a lidar
    //store distance from position to right wall
    //and also store distance from current position to distance ahead
    // compare the distances with unstored calculated left distance
    //the robot moves in the direction with longest distance
    (long distance simply implies that no wall detected)
    */ 
    bool isGo = false;

}
void turnLeft()//turn left
{

}
void turnRight()//go right
{

}