/* The following function displays the FloorPlan with the grid 
   
   1. drawFloorPlan(grid = true) */
void drawFloorPlan(boolean grid) 
{
  // Draw the floorplan 
  image(img, 0, 0, width/1.02, height/1.3);
  
  /* Set the boolean value in the function parameter to true 
     to view the forbidden zones (where people cannot be) */
  //showForbiddenZones(true);
  
  if(grid)
  {
    // Overlay the grid on top of the floorplan
      for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        int x = i * cellSize;
        int y = j * cellSize; 
      
        // Draw grid squares
        noFill();  // No fill color
        stroke(0, 50);  // Light grid lines
        rect(x, y, cellSize, cellSize);  // Draw the square
      }
    }
  }
  
}


/* The following functions are to handle the forbidden zones on the Map

   1. void showForbiddenZones(boolean val)
   2. boolean isInForbiddenZone(int x, int y, int size)  */

void showForbiddenZones(boolean val)
{
  
  if(val)
  {
    noStroke();
    fill(255, 0, 0, 100);  // Translucent red for forbidden zones
    rect(0 * cellSize, (rows-5) * cellSize, 68 * cellSize, 5 * cellSize);
    rect(0 * cellSize, 0 * cellSize, 13 * cellSize, 24 * cellSize);
    rect(13 * cellSize, 0 * cellSize, (cols-13) * cellSize, 5 * cellSize);
    rect(28 * cellSize, 5 * cellSize, 15 * cellSize, 19 * cellSize);
    rect((cols-10) * cellSize, 5 * cellSize, 10 * cellSize, 19 * cellSize);
    rect((cols-2) * cellSize, 24 * cellSize, 2 * cellSize, 9 * cellSize);
    
    rect(13 * cellSize, 19 * cellSize, 4 * cellSize, 5 * cellSize);
    rect(24 * cellSize, 19 * cellSize, 4 * cellSize, 5 * cellSize);
    rect(43 * cellSize, 19 * cellSize, 4 * cellSize, 5 * cellSize);
    rect(54 * cellSize, 19 * cellSize, 4 * cellSize, 5 * cellSize);
    
    rect(0 * cellSize, 24 * cellSize, 3 * cellSize,  9* cellSize);
    rect(3 * cellSize, 24 * cellSize, 4 * cellSize,  1* cellSize);
    rect(3* cellSize, (rows-8) * cellSize, 3 * cellSize, 3 * cellSize);
    rect(3* cellSize, (rows-11) * cellSize, 1 * cellSize, 3 *cellSize);
    rect(4* cellSize, (rows-10) * cellSize, 1 * cellSize, 2 *cellSize);
    rect(6* cellSize, (rows-7) * cellSize, 1 * cellSize, 2 *cellSize);
    rect(7* cellSize, (rows-6) * cellSize, 1 * cellSize, 1 *cellSize);
  }
}

boolean isInForbiddenZone(int x, int y, int size) {
  // Forbidden Zone 1
  if (x >= 0 * size && x < 68 * size && y >= (rows - 5) * size && y < rows * size) {
    return true;
  }
  // Forbidden Zone 2
  if (x >= 0 * size && x < 13 * size && y >= 0 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 3
  if (x >= 13 * size && x < cols * size && y >= 0 * size && y < 5 * size) {
    return true;
  }
  // Forbidden Zone 4
  if (x >= 28 * size && x < 43 * size && y >= 5 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 5
  if (x >= (cols - 10) * size && x < cols * size && y >= 5 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 6
  if (x >= (cols - 2) * size && x < cols * size && y >= 24 * size && y < 33 * size) {
    return true;
  }
  // Forbidden Zone 7
  if (x >= 13 * size && x < 17 * size && y >= 19 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 8
  if (x >= 24 * size && x < 28 * size && y >= 19 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 9
  if (x >= 43 * size && x < 47 * size && y >= 19 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 10
  if (x >= 54 * size && x < 58 * size && y >= 19 * size && y < 24 * size) {
    return true;
  }
  // Forbidden Zone 11
  if (x >= 0 * size && x < 3 * size && y >= 24 * size && y < 33 * size) {
    return true;
  }
  // Forbidden Zone 12
  if (x >= 3 * size && x < 7 * size && y >= 24 * size && y < 25 * size) {
    return true;
  }
  // Forbidden Zone 13
  if (x >= 3 * size && x < 6 * size && y >= (rows - 8) * size && y < (rows - 5) * size) {
    return true;
  }
  // Forbidden Zone 14
  if (x >= 3 * size && x < 4 * size && y >= (rows - 11) * size && y < (rows - 8) * size) {
    return true;
  }
  // Forbidden Zone 15
  if (x >= 4 * size && x < 5 * size && y >= (rows - 10) * size && y < (rows - 8) * size) {
    return true;
  }
  // Forbidden Zone 16
  if (x >= 6 * size && x < 7 * size && y >= (rows - 7) * size && y < (rows - 5) * size) {
    return true;
  }
  // Forbidden Zone 17
  if (x >= 7 * size && x < 8 * size && y >= (rows - 6) * size && y < (rows - 5) * size) {
    return true;
  }

  // Not in any forbidden zone
  return false;
}


/* The following function utilizes the mouse click to fix a starting position
   1. mouseClicked() // Default Processing function */

void mouseClicked()
{
  if(!ignoreMouseClicks)
  {
    startX = (mouseX / cellSize) * cellSize;
    startY = (mouseY / cellSize) * cellSize;  
    
    if(isInForbiddenZone(startX, startY, cellSize))
    {
      startX = 0;
      startY = 0;
      
      ignoreMouseClicks = false;
    }
    else
    {
      currentX = startX;
      currentY = startY;      
      ignoreMouseClicks = true;
    }
  }
}



/* The following function finds out the direction of the next step on the basis of yaw
   1. nextStepDirection(float yaw) */

void nextStepDirection(float yaw) {
  if ((yaw >= 327.5 && yaw < 360) || (yaw >= 0 && yaw < 12.5)) {
    // Move Up (North)
    type = 2;
    nextX = ((currentX / cellSize)) * cellSize;
    nextY = ((currentY / cellSize) - 1) * cellSize;
  } else if (yaw >= 12.5 && yaw < 57.5) {
    // Move Up-Right (North-East)
    type = 3;
    nextX = ((currentX / cellSize) + 1) * cellSize;
    nextY = ((currentY / cellSize) - 1) * cellSize;
  } else if (yaw >= 57.5 && yaw < 102.5) {
    // Move Right (East)
    type = 4;
    nextX = ((currentX / cellSize) + 1) * cellSize;
    nextY = ((currentY / cellSize)) * cellSize;
  } else if (yaw >= 102.5 && yaw < 147.5) {
    // Move Down-Right (South-East)
    type = 5;
    nextX = ((currentX / cellSize) + 1) * cellSize;
    nextY = ((currentY / cellSize) + 1) * cellSize;
  } else if (yaw >= 147.5 && yaw < 192.5) {
    // Move Down (South)
    type = 6;
    
    nextX = ((currentX / cellSize)) * cellSize;
    nextY = ((currentY / cellSize) + 1) * cellSize;
  } else if (yaw >= 192.5 && yaw < 237.5) {
    // Move Down-Left (South-West)
    type = 7;
    nextX = ((currentX / cellSize) - 1) * cellSize;
    nextY = ((currentY / cellSize) + 1) * cellSize;
  } else if (yaw >= 237.5 && yaw < 282.5) {
    // Move Left (West)
    type = 8;
    nextX = ((currentX / cellSize) - 1) * cellSize;
    nextY = ((currentY / cellSize)) * cellSize;
  } else if (yaw >= 282.5 && yaw < 327.5) {
    // Move Up-Left (North-West)
    type = 1;
    nextX = ((currentX / cellSize) - 1) * cellSize;
    nextY = ((currentY / cellSize) - 1) * cellSize;
  }
}





/* The following functions checks for stairs (only one zone covered) 
   
   1. boolean CheckStairs(int x, int y, int size) 
   2. void StairsHandler(int x, int y)  */
   
boolean CheckStairs(int x, int y, int size)
{
  if (x >= 47 * size && x < 52 * size && y >= 20 * size && y < 24 * size)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void StairsHandler(int floor)
{
  if(CheckStairs(currentX, currentY, cellSize))
  {
    if(floor != 1)
    {
      if(floor != 2)
      stepLength = 0.3;
    }
  }
}


/* The following function handles the floor depending on the altitude
   
   1. void FloorHandler(float altitude) */
   
int FloorHandler(float alt)
{
  if(alt > (0.98*Floor1) && alt < (0.98 *Floor2))
  {
    return 1;
  }
  
  else if(alt > (0.98*Floor2))
  {
    return 2;
  }
  
  else
  {
    return 0;
  }
}



/* The following function is responsible for taking data from the Serial port
   
   1. void SerialEvent(Serial port) */

void serialEvent(Serial port) {
  // Read the data until a newline character
  String receivedData = port.readStringUntil('\n');
  
  if (receivedData != null) {
    receivedData = trim(receivedData); // Remove extra whitespace or newline
    
    // Split the received data by commas
    String[] values = split(receivedData, ',');
    
    // Ensure the correct number of values is received
    if (values.length == 4) {
      try {
        // Parse the values
        step = values[0].equals("1");               // Convert string to boolean
        yaw = float(values[1]);                     // Convert to float
        stepLength = float(values[2]);              // Convert to float
        altitude = int(values[3]);                // Convert to float
        
        
        // Debugging output
        /*println("Step: " + step);
        println("Yaw: " + yaw);
        println("Step Length: " + stepLength);
        println("Altitude: " + altitude);*/
        

      } catch (Exception e) {
        println("Error parsing data: " + e.getMessage());
      }
    } else {
      println("Incorrect data format received: " + receivedData);
    }
  }
}


/* The following function just displays the floor on a nice display

   1. void DisplayFloor() */
   
void DisplayFloor()
{
  fill(204, 204, 204);
  noStroke();
  rect(5,775,350,60);
  
  textSize(50);
  fill(0, 0, 128);
  text("Current Floor: ", 10, 820);
  
  String CurFloor = str(currentFloor);
  
  fill(0, 0, 0);
  text(CurFloor, 310, 820);
}

void DisplayLocation()
{
  fill(204, 204, 204);
  noStroke();
  rect(5,850,500,60);
  
  textSize(50);
  fill(0, 0, 128);
  text("Location: ", 10, 900);
  
  String CurLoc = "50.77N, 6.04E";
  
  fill(0, 0, 0);
  text(CurLoc, 215, 900);
}
