/*
 * Organizer.cpp
 */
 
void waitForButtonPress()
{
  // Serial.println("Press button to start moving");

  do
  {
    currentMillis = millis();
    
    flashLedG(); // Flashes LEDg to indicate waiting for button press before moving
    
    buttonState = digitalRead(buttonPin);
    
  } while (buttonState == HIGH); // While button is not pressed

  // Serial.println("button pressed");
}

void flashLedG() 
{
  if (currentMillis - prevLedGMillis >= ledGInterval) 
  {
    prevLedGMillis += ledGInterval; // This updates the timing ready for the next interval (more accurately than using prevLedGMillis = currentLedMillis)
    
    ledGState = ! ledGState;
    
    if (ledGState == HIGH)
      ledGInterval = ledOnMillis;
      
    else
      ledGInterval = ledGBaseInterval;
    
    digitalWrite(ledGPin, ledGState);
  }
}

void flipLedG() 
{
  ledGState = ! ledGState;
  
  digitalWrite(ledGPin, ledGState);
}

void flipLedR() 
{
  ledRState = ! ledRState;
  
  digitalWrite(ledRPin, ledRState);
}

void askGetAck()
{
  askGetAckSettings();
}

void askGetAckSettings()
{
//  if (useDefaultSettings == true)
//  {
//    return;
//  }

  for (byte i = 0; i < numInputs; i++)
  {
    do
    {
      askQuestion(i);
      getResponse(i);
    } while (ackResponse == false);
    acknowledgeResponse(i);
  }
}

void askQuestion(byte qNum)
{
  if (waitingForResponse == true)
  {
    return;
  }

  // (else clause unnecessary because of use of return above)
  question = questions[qNum]; // get the array address
  Serial.println(question);
  waitingForResponse = true;
  /* The end of an ask function (i.e. here) is the best place to reset received bytes,
     just in case other parts of the code need to know # of bytes actually received */
  bytesRecvd = 0;
}

void getResponse(byte rNum)
{
  if (waitingForResponse == false)
  {
    return;
  }

  if (Serial.available() == 0)
  {
    return;
  }

  char inChar = Serial.read();

  if (rNum == 0)
  { // get x target
    if (inChar != endMarker)
    {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseXCoordinate[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize)
      {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else
    { // inChar is the endMarker
      xCoordinate = input;
      xTarget = (float)xCoordinate;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseXCoordinate[bytesRecvd] = 0;
      ackResponse = true;
    }
  } else if (rNum == 1)
  { // get y target
    if (inChar != endMarker)
    {
      int c = (int)inChar - 48;
      input *= 10;
      input += c;
      userResponseYCoordinate[bytesRecvd] = inChar;
      bytesRecvd ++;
      if (bytesRecvd == buffSize)
      {
        bytesRecvd = buffSize - 1; // because of 0 that compiler auto adds
      }
    } else
    { // inChar is the endMarker
      yCoordinate = input;
      yTarget = (float)yCoordinate;
      input = 0; // Reset input for next question
      waitingForResponse = false;
      userResponseYCoordinate[bytesRecvd] = 0;
      ackResponse = true;
    }
  }
}

void resetCoordinates()
{
  xPos = 0.0;
  yPos = 0.0;
  theta = 0.0;
}
