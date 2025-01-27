
/* The following function displays the FloorPlan with the grid 
   
   1. drawFloorPlan(grid = true) */
void drawFloorPlan(boolean grid) 
{
  // Draw the floorplan 
  image(img, 0, 0, width/1.02, height/1.3);
  
  if(grid)
  {
    // Overlay the grid on top of the floorplan
      for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        int x = i * cellSize;
        int y = j * cellSize; 
      
        // Draw grid squares
        noFill();  // No fill color
        stroke(0, 30);  // Light grid lines
        rect(x, y, cellSize, cellSize);  // Draw the square
      }
    }
  }
  
}


/* The following function handles the floor depending on the altitude
   
   1. int FloorHandler() */
   
int FloorHandler()
{

  float difPres = abs( int(pressure - groundPress) );
  
  if(difPres >= 20 && difPres < 65) 
  {
    return 1;
  }
  
  else if(difPres >= 65 && difPres < 110)
  {
    return 2;
  }
  
  else
  {
    return 0;
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


// Display the grid square that the mouse is currently located in

void getCurrentMouseLocation()
{
  //println( (int(mouseX/cellSize)), (int(mouseY/cellSize)));
  gridx = (int(mouseX/cellSize));
  gridy = (int(mouseY/cellSize));
}


void calculateProbabilities(float yawAngle, float sigma, float sigmaDistance, int startx, int starty, int gridsize)
{  
  for(int x = startx - gridsize/2, i=0; x < startx + gridsize/2; x++, i++)
  {
    for(int y = starty - gridsize/2, j=0; y < starty + gridsize/2; y++, j++)
    {
      
      float dx = x - startx;  // Horizontal offset
      float dy = y - starty;  // Vertical offset
      float distance = sqrt(dx*dx + dy*dy);  // Calculate euclidian distance
      
      float angle = atan2(dy, dx);  // Angle of cell relative to the center (current position)
      
      float angleDiff = normalizeAngle(angle - yawAngle);  // wrap to [0, 2pi] 
      
      float distanceProb = exp(-sq(distance) / (2 * sq(sigmaDistance)));  // Distance decay
      
      if(abs(angleDiff) > 3*sigma)
      {
        probabilities[i][j] = 0;
      }
      else
      {
        // Assign gaussian probability based on angular difference
        // The weights are empirically determined. Gaussian probability should have lower priority than distance
        probabilities[i][j] = 0.25*gaussian(angleDiff, sigma) + 0.75*distanceProb;  
      }
      
      applyWallBlocking(startx, starty, gridsize);
      
      probabilities[5][5] = 0;

    }
  }
}

float gaussian(float x, float sigma)
{
  return (1.0 / (sigma * sqrt(TWO_PI))) * exp(-0.5 * sq(x / sigma));
}

// Ensure probabilities sum up to 1
void normalizeProbabilities(int startx, int starty, int gridsize)
{
  float sum = 0;
  for (int x = startx - gridsize/2, i = 0; x < startx + gridsize/2; x++, i++) {
    for (int y = starty - gridsize/2, j = 0; y < starty + gridsize/2; y++, j++) {
      sum += probabilities[i][j];
    }
  }
  for (int x = startx - gridsize/2, i=0; x < startx + gridsize/2; x++, i++) {
    for (int y = starty - gridsize/2, j=0; y < starty + gridsize/2; y++, j++) {
      probabilities[i][j] /= sum;
    }
  }
}

void drawProbabilities(int startx, int starty, int gridsize, boolean drawAllowed) {
  
  if(drawAllowed)
  {
    for (int x = startx - gridsize/2, i=0; x < startx + gridsize/2; x++, i++) {
    for (int y = starty - gridsize/2, j=0; y < starty + gridsize/2; y++, j++) {
      float intensity = map(probabilities[i][j], 0, 1, 0, 1200);
     
      noStroke();
      fill(255, 0, 0, intensity);
      rect(x * cellSize, y * cellSize, cellSize, cellSize);
      }
    }
  }
}

float normalizeAngle(float angle) {
  
  angle = angle + PI/2 - PI/9;    // Aligning magnetic fields to the Map in the special case of Helmholtz 
  
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}


// Function to load the walls 
void loadWalls(String filePath) {
  String[] lines = loadStrings(filePath);
  for (String line : lines) {
    String[] coords = line.split(",");
    int x = int(trim(coords[0]));
    int y = int(trim(coords[1]));
    walls.put(x + "," + y, true);
  }
}

// Function to display the walls
void displayWalls(HashMap<String, Boolean> walls, int cellSize) {
  fill(140); // Wall color (gray)
  noStroke(); // No border for wall cells

  for (String key : walls.keySet()) {
    String[] coords = key.split(",");
    int x = int(coords[0]);
    int y = int(coords[1]);

    // Draw the wall cell
    rect(x * cellSize, y * cellSize, cellSize, cellSize);
  }
}


// Use Bresenham's algorithm to check if the ray passes through any wall squares
boolean isBlocked(int startx, int starty, int targetX, int targetY) {
  int dx = abs(targetX - startx);
  int dy = abs(targetY - starty);
  int sx = (startx < targetX) ? 1 : -1;
  int sy = (starty < targetY) ? 1 : -1;

  int err = dx - dy;

  int x = startx;
  int y = starty;

  while (true) {
    // Check if current square is a wall
    if (walls.containsKey(x + "," + y)) {
      return true; // Ray is blocked
    }

    // Reached the target square
    if (x == targetX && y == targetY) {
      break;
    }

    int e2 = 2 * err;

    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }

    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return false; // No wall blocks the ray
}

void applyWallBlocking(int startx, int starty, int gridSize) {
  for (int x = startx - gridSize / 2, i = 0; x < startx + gridSize / 2; x++, i++) {
    for (int y = starty - gridSize / 2, j = 0; y < starty + gridSize / 2; y++, j++) {

      if (probabilities[i][j] > 0) {
        if (isBlocked(startx, starty, x, y)) {
          probabilities[i][j] = 0;
        }
      }
      
    }
  }
}

// Step algorithm (probabilities is the 10x10 mini-grid and the goal of the function is to find the highest probability square inside the step radius)
PVector findHighestProbabilityInCircle(float[][] probabilities, int centerX, int centerY, float stepLength, int gridSize) {
  float squareSize = 10.0 / 17.0; // Real-world meters per square
  int maxRadius = ceil(stepLength / squareSize); // Convert step length to grid radius

  float maxProbability = -1; // Start with an invalid probability
  PVector bestSquare = new PVector(centerX, centerY); // Default to the current position

  // Iterate over all squares in a circular range
  for (int dx = -maxRadius; dx <= maxRadius; dx++) {
    for (int dy = -maxRadius; dy <= maxRadius; dy++) {
      int x = centerX + dx;
      int y = centerY + dy;

      // Check if (x, y) is within the grid bounds
      int localX = dx + gridSize / 2;
      int localY = dy + gridSize / 2;
      
      
      if (localX >= 0 && localX < probabilities.length && localY >= 0 && localY < probabilities[0].length) {
        // Check if (x, y) lies within the circular radius
        float distance = sqrt(dx * dx + dy * dy);
        if (distance <= maxRadius) {
          // Compare probabilities to find the highest
          if (probabilities[localX][localY] > maxProbability) {
            maxProbability = probabilities[localX][localY];
            bestSquare.set(x, y); // Update the best square
          }
        }
      }
    }
  }

  return bestSquare; // Return the best square
}



void updatePositionWithProbabilities(float[][] probabilities, int centerX, int centerY, float stepLength) {
  
  PVector bestSquare = findHighestProbabilityInCircle(probabilities, centerX, centerY, stepLength, gridsize);
  
  currentX = int(bestSquare.x);
  currentY = int(bestSquare.y);
}

void drawCurrentPosition(int currentX, int currentY, int cellSize, boolean drawAllowed)
{ 
  if(drawAllowed)
  {
    fill(0, 0, 0);
    noStroke();
    rect(currentX * cellSize, currentY * cellSize, cellSize, cellSize);
  }
}

void updateProbabilitiesForWalls(float[][] probabilities, HashMap<String, Boolean> walls) {
  for (int x = 0; x < probabilities.length; x++) {
    for (int y = 0; y < probabilities[0].length; y++) {
      // Check if the current square is a wall
      String key = x + "," + y;
      if (walls.containsKey(key)) {
        probabilities[x][y] = 0; // Set probability to 0 for walls
      }
    }
  }
}


void displayStartPosition(int startx, int starty)
{
  fill(0, 0, 100);
  noStroke();
  rect(startx *cellSize, starty *cellSize, cellSize, cellSize);  
}


void CorrectError(int x, int y, int cellsize)
{
  int counter = 0;
  
  for(int i = 0; i<cellsize; i++)
  {
    for(int j=0; j<cellsize; j++)
    {
      if(Float.isNaN(probabilities[i][j]))
      {
        counter++;
      }
    }
  }
  
  // When we are stuck in x or y direction, we jump back to a fixed line along the axis of the corridor
  
  // Zone 1  
  if(counter == 100 && x > 24 && x < 57 && y > 9 && y < 53)
  {
    currentX = 37;
  }
  
  // Zone 2
  if(counter == 100 && x > 3 && x < 135 && y > 46 && y < 67)
  {
    currentY = 56;
  }
  
  // Zone 3
  if(counter == 100 && x > 84 && x < 117 && y > 9 && y < 53)
  {
    currentX = 105;
  }
}


void serialReadThread() {
  while (true) { // Infinite loop to continuously read data
    if (myPort.available() > 0) {
      String receivedData = myPort.readStringUntil('\n');
      if (receivedData != null) {
        receivedData = trim(receivedData); // Remove extra whitespace or newline

        // Split the received data by commas
        String[] values = split(receivedData, ',');

        // Ensure the correct number of values is received
        if (values.length == 5) {
          try {
            synchronized (this) { // Ensure thread-safe access
              stepTaken = (values[0]).equals("1"); // Convert string to boolean
              yaw = float(values[1]);             // Convert to float
              stepLength = float(values[2]);      // Convert to float
              pressure = float(values[3]);        // Convert to float
              RSSI = (float(values[4]));
              
              newDataAvailable = true;            // Signal new data availability
              
              if(!firstPressTaken)
              {    
                groundPress = pressure;
                
                firstPressTaken = true;
              }
            }
          } catch (Exception e) {
            println("Error parsing data: " + e.getMessage());
          }
        } else {
          println("Incorrect data format received: " + receivedData);
        }
      }
    }
    delay(10); // Add a slight delay to prevent CPU overload
  }
}

void HeatMapButtonHandler()
{
  int colorChanger;
  
  if(mouseX > 1060 &&  mouseX < (1060 +250) && mouseY > 790 && mouseY < (790 + 60))
  {
    colorChanger = 255;
    overHeatMapButton = true;
  }
  else
  {
    colorChanger = 204;
    overHeatMapButton = false;
  }
  
  
  fill(colorChanger, colorChanger, colorChanger);
  stroke(0,0,0);
  rect(1060, 790, 250,60);
  
  textSize(30);
  fill(0, 0, 128);
  text("Display HeatMap", 1080, 830);
}

void drawApproximateLocation()
{
  // Disable drawing the Approximate Blue Circle if Test Mode is Enabled for better visibility
  if(!EnableTestMode)
  {
    if(millis() - displayTime >= 5000)
    {
      drawAllowed = false;
      
      noStroke();
      fill(0, 0, 255, 150);
      circle(currentX * cellSize, currentY * cellSize, 50);
    }
  }
}

void updateHeatMap(float[][] probabilities, int currentx, int currenty, int gridsize)
{
  for(int i = 0; i < gridsize; i++)
  {
    for(int j = 0; j < gridsize; j++)
    {
      if(HeatMap[currentx - gridsize + i][currenty - gridsize + j] <= probabilities[i][j])
      {
        HeatMap[currentx - gridsize + i][currenty - gridsize + j] = probabilities[i][j];     
      }
        
    }
  } 
}

void drawHeatMap()
{
  for(int i = 0; i < cols; i++)
  {
    for(int j = 0; j < rows; j++)
    {
      noStroke();
      fill(255, 0, 0, map(HeatMap[i][j], 0, 1, 0, 1200));
      rect((i+5) * cellSize, (j+5) *cellSize, cellSize, cellSize);
    }
  }
}
