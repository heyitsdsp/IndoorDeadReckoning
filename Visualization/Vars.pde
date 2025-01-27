// Serial parameters
Serial myPort;              // Declare object for the serial port
String receivedData = "";   // Store incoming data

// Grid parameters
int cols = 137;
int rows = 77;
int cellSize = 10;

// Navigating the grid
int gridx = 0;
int gridy = 0;

int startX = 0;
int startY = 0;
boolean startPositionChosen = false;

int currentX = 0;
int currentY = 0;

// Yaw handling parameters
volatile float yaw = radians(350.0);
float sigma = PI/18;  // 10 degree spread
float sigmaDistance = 1;
int gridsize = 10;
float[][] probabilities = new float[gridsize][gridsize];

// Wall handling parameters
HashMap<String, Boolean> walls = new HashMap<>();

// Step handling parameters
volatile boolean stepTaken = false;
volatile float stepLength = 0.8;

Thread dataThread;
String incomingData;
boolean newDataAvailable = false;

long displayTime = 0;
boolean drawAllowed = false;

// HeatMap settings
boolean displayHeatMap = false;
boolean overHeatMapButton = false;
float[][] HeatMap = new float[cols][rows];

// RSSI for the BLE based recalibration
float RSSI = -70;

// altitude and floor parameters
int currentFloor = 0;
volatile float pressure = 0;
float groundPress = 0;
boolean firstPressTaken = false;
