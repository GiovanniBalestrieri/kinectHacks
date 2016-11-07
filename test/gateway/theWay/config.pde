

// Show Gateway
boolean showGatewayPoint = false;
boolean showGatewayArrow = false;


int skipPx = 2;

// Robot Width
int robotWidth = 300;
// Obstacle dimension for horizon segmentation 
int obstacleHorizonThreshold = 20;

float depthThreshold = 1.0;

int rangeDetection = 2;

int     maxEdges=800;

float[] edges = new float[maxEdges];

float[] edgesDx = new float[maxEdges];

float[] edgesSx = new float[maxEdges];

// Verbosity

boolean VERBOSE_MOTOR = false;