import processing.serial.*;

PImage img;

// Serial parameters
Serial myPort;              // Declare object for the serial port
String receivedData = "";   // Store incoming data

// Grid parameters
int cols = 68;
int rows = 38;
int cellSize = 20;

// Step parameters
int currentX = 0;
int currentY = 0;

int nextX = 0;
int nextY = 0;

int startX = 0;
int startY = 0;

boolean inForbiddenZone = false;

float yaw = 0;

boolean step = false;
float stepLength = 0.8;
double distance = 0;

boolean ignoreMouseClicks = false;

int type = 0;
int lastType = 0;

float gridSquareSide = 1.39;  // Change this. It's currently set to 1m for the Side of the square. 
int squaresMoved = 0;
int squaresMovedPrev = 0;

// altitude and floor parameters
int currentFloor = 0;
int Floor1 = 301;
int Floor2 = 305;
int altitude = 0;

void setup() {
  size(1400, 1000);
  img = loadImage("Helmholtz_FloorPlan.png");  
  
  myPort = new Serial(this, "COM5", 9600);
  myPort.bufferUntil('\n');
  
}


void draw() {
  
  drawFloorPlan(true);
  
  // Choose starting point
  if(currentX ==0 && currentY ==0)
  {
    
  }
  
  // After starting point has been chosen
  else
  {     
    // Color the starting point blue
    fill(0, 0, 200);
    rect(startX, startY, cellSize, cellSize);
  }
  
  nextStepDirection(yaw);
  
  if(type != lastType)
  {
    distance = 0;
    
    lastType = type;
  }
  else
  {
    if(step)
    {
      distance += stepLength;
      
      switch(type % 2) {
        
        case 0: 
          squaresMoved = (int) (distance / gridSquareSide);          
          break;
          
        case 1:
          squaresMoved = (int) (distance / (sqrt(2) * gridSquareSide));    
          break;
      }
      
      drawNextFrame(squaresMoved);
      
      step = false;
    }
  }
  
  fill(255, 50, 50, 150);
  rect(currentX, currentY, cellSize, cellSize);
  
  StairsHandler(currentFloor); 
  
  currentFloor = FloorHandler(altitude);
  
  println(altitude);
  
  
  DisplayFloor();
  DisplayLocation();
}


void drawNextFrame(int squaresMoved)
{
  if(squaresMoved - squaresMovedPrev == 1)
  {
    // draw next frame
    if(! isInForbiddenZone(nextX, nextY, cellSize))
    {
      currentX = nextX;
      currentY = nextY;
    }
    
  }
  
  squaresMovedPrev = squaresMoved;
  
}



// Testbench / Debug mode
void keyPressed() {
  if(keyCode == UP)
  step = true;
  
  if (key == 'a') {
    yaw += 5; // Increment yaw by 5 degrees
    if (yaw >= 360) {
      yaw -= 360; // Rollover at 360 degrees
    }
  }
  
  if (key == 'f'){
    altitude += 10;
  }
  if (key == 'g'){
    altitude -= 10;
  }
  
} 
