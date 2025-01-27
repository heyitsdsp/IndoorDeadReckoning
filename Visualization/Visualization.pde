import processing.serial.*;

PImage img;

void setup() {
  size(1400, 1000);
  img = loadImage("Helmholtz_FloorPlan.png");  
  
  //Load the walls from walls.txt
  loadWalls("walls.txt");
  
  // Limit framerate to not overload the Processing Application
  frameRate(30);
  
  // If TestMode is disabled, then take the data from COM Port
  if(!EnableTestMode)
  {
    myPort = new Serial(this, "COM5", 115200);
  
    thread("serialReadThread");
  }
  
}


void draw() {
  
  drawFloorPlan(true);
  
  // Display the Heat Map of probabilities
  if(displayHeatMap)
  {
    drawHeatMap();
  }
  
  // When beacon is near, we set the location to the beacon location in the grid (-46 is the threshold)
  if(RSSI > -46)
  {
    currentX = 41;
    currentY = 15;
  }
  
  
  // Show the correction zones if needed (displays the rectangles that we consider for error correction)
  if(ShowCorrectionZones)
  {  
    fill(0, 0, 255, 150);
    rect(24 * cellSize, 9 *cellSize, 34 * cellSize, 45* cellSize);
    
    fill(0, 0, 255, 150);
    rect(3 * cellSize, 46 *cellSize, 133 * cellSize, 22* cellSize);
    
    fill(0, 0, 255, 150);
    rect(84 * cellSize, 9 *cellSize, 34 * cellSize, 45* cellSize); 
  }
    
 
  if(startPositionChosen)
  {
    //display Start Position
    displayStartPosition(startX, startY);
    
    // Display the walls
    if(ShowWalls)
    {
      displayWalls(walls, cellSize);
    }
    
    if(newDataAvailable)
    {
      synchronized(this)
      {
        
        if(stepTaken)
        {
          updatePositionWithProbabilities(probabilities, currentX, currentY, stepLength);
          
          CorrectError(currentX, currentY, cellSize);
          
          stepTaken = false;
          
          drawAllowed = true;
        }
        
    newDataAvailable = false;
    
    displayTime = millis();
    
      }
    }
    
  // Handle the probability distribution  
  calculateProbabilities(yaw, sigma, sigmaDistance, currentX, currentY, gridsize);
  
  updateProbabilitiesForWalls(probabilities, walls);
  
  normalizeProbabilities(currentX, currentY, gridsize);
  
  // Draw probabilities and current position
  drawProbabilities(currentX, currentY, gridsize, drawAllowed);
  
  drawCurrentPosition(currentX, currentY, cellSize, drawAllowed);
  
  // Handle Floors by factoring in the Pressure
  currentFloor = FloorHandler();
  DisplayFloor();
  
  // Experimental for GPS integration
  DisplayLocation();
  
  // Click the button in the GUI to show the HeatMap of probabilities
  HeatMapButtonHandler();
  
  // Blue circle after 5 seconds
  drawApproximateLocation();
  
  // Update HeatMap for every step taken
  updateHeatMap(probabilities, currentX, currentY, gridsize);
  }
  
}

void keyPressed()
{
  newDataAvailable = true;
  
  if(key == 'w' || key == 's')
  {
    // Press "w" to increase yaw angle and "s" to decrease yaw angle
    if(key == 'w')
    {
      yaw = yaw + radians(2.0);
    }
    else
    {
      yaw = yaw - radians(2.0);
    }
    
    // Wrap the angles from -2pi to 2pi
    if(yaw >= TWO_PI)
    {
      yaw -= TWO_PI;
    } 
  }

  // Take steps using the keyboard. Press spacebar to take a step
  if(key == ' ')
  {
    stepTaken = true;
  }
  
  // Demo value of RSSI to recalibrate. Press "r" to set the RSSI value (can be done only once)
  if(key == 'r')
  {
    RSSI = -35;
  }
  
}


// Processing's in-built mouseClick function - to choose the start position and implement the HeatMap button functionality
void mouseClicked()
{
  if(!startPositionChosen)
  {
    getCurrentMouseLocation();
    
    currentX = gridx;
    currentY = gridy;
    
    startX = currentX;
    startY = currentY;
    
    startPositionChosen = true;
  }
  
  if(overHeatMapButton)
  {
    startPositionChosen = false;
    displayHeatMap = true;
  }
}
