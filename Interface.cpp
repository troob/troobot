/*
 * Interface.cpp
 */
 

void giveInfo()
{
  Serial.println("This is location (x, y).");
  delay(1000);
}

void acknowledgeResponse(byte rNum)
{
  if (ackResponse == false)
  {
    return;
  }

  if (rNum == 0)
  { // acknowledge x coordinate
    Serial.print("The x coordinate will be ");
    Serial.print(userResponseXCoordinate);
    if (xCoordinate == 1)
    {
      Serial.println(" centimeter [cm].");
    } else
    {
      Serial.println(" centimeters [cm].");
    }
  } else if (rNum == 1)
  { // acknowledge y coordinate
    Serial.print("The y coordinate will be ");
    Serial.print(userResponseYCoordinate);
    if (yCoordinate == 1)
    {
      Serial.println(" centimeter [cm].");
    } else
    {
      Serial.println(" centimeters [cm].");
    }
  }

  Serial.println();

  ackResponse = false;
}
