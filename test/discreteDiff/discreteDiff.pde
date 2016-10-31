import org.openkinect.processing.*;
import peasy.*;


PeasyCam cam;

Kinect kinect;
PImage img;
int[] depth;
float angle;
int skip ;
int skipMes = 14;
// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
float bigDist = 0.30f, smallDist = 0.07f;
int row = 350, row1 = 370;
ArrayList<CornerLeft> listaCornerLeft = new ArrayList<CornerLeft>();
ArrayList<CornerRight> listaCornerRight = new ArrayList<CornerRight>();

// Store laserScans
ArrayList<PVector[]> scans = new ArrayList<PVector[]>();


float[] corners = new float[10];
MedianFilter medLine = new MedianFilter((byte)3,0);

PVector[] filtLine1;
PVector[] filtLine2;
PVector[] filtLine3;
int contSkip = 0, historyClean = 0;
int frameRateKinect = 20;
PVector maxOnLine = new PVector(0,0,0);

boolean drawSceneB = true, drawCornersB = true, drawMaxB = false;

void setup(){
  // Rendering in P3D
  size(800, 600, P3D);
  
  frameRate(frameRateKinect);
  
  cam = new PeasyCam(this, 100);
  cam.setMinimumDistance(0);
  cam.setMaximumDistance(1500);
  cam.setSuppressRollRotationMode();
  
  kinect = new Kinect(this);
  kinect.initDepth();
  skip = 7;
  angle = kinect.getTilt();
 
  depthLookUp = new float[2048];
  for (int i = 0; i < depthLookUp.length; i++) 
  {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  /*
  println(" f "+ kinect.width + " ; " + kinect.height);
  filtLine1 = new int[(int) (kinect.width/skip)+1];
  filtLine2 = new int[(int) (kinect.width/skip)+1];
  filtLine3 = new int[(int) (kinect.width/skip)+1];
  */
  println(" f "+ kinect.width + " ; " + kinect.height);
  filtLine1 = new PVector[(kinect.width)];
  filtLine2 = new PVector[(kinect.width)];
  filtLine3 = new PVector[(kinect.width)];
  
  scans.add(filtLine1);
  scans.add(filtLine3);
  scans.add(filtLine2);
}

void draw(){
  background(0);
  
  
  depth = kinect.getRawDepth(); 
  // Translate and rotate
  translate(0,0,0);
  stroke(255);
  beginShape(POINTS);
  strokeWeight(2);
  saveRowsDepth();
  cleanCornersMemory();
  findCorners();  
  drawRoutine();
  
  updateCounters();
}

void drawRoutine()
{
  if (drawSceneB)
    drawScenePCL();
  if (drawCornersB)
    drawCorners();
  if (drawMaxB && findMaximum(filtLine1))
  {
    drawMax(maxOnLine);
  }
}

void drawMax(PVector v)
{
  strokeWeight(15);
  stroke(100,0,100);
  pushMatrix();
    // Scale up by 200S
    float factor = 200;
    translate(v.x * factor, v.y * factor, factor-v.z * factor);
    // Draw a point
    point(0,0);
  popMatrix();
}

void updateCounters()
{
  historyClean ++;
}

void cleanCornersMemory(){
  if (true)
  {
      // If you are modifying an ArrayList during the loop,
      // then you cannot use the enhanced loop syntax.
      // In addition, when deleting in order to hit all elements, 
      // you should loop through it backwards, as shown here:
      for (int i = listaCornerLeft.size() - 1; i >= 0; i--) {
        listaCornerLeft.remove(i);
      } 
      for (int i = listaCornerRight.size() - 1; i >= 0; i--) {
        listaCornerRight.remove(i);
      } 
      historyClean = 0;
  }
}

void drawScenePCL(){  
 println("CL: " + listaCornerLeft.size()+ " CR: " + listaCornerRight.size());
  
 for(int x = 0; x < kinect.width ; x += skip) {   
    for(int y = 0; y < kinect.height; y += skip) {         
      int offset = x + y * kinect.width;
      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      
      strokeWeight(2);
      stroke(255);  
      
      if (y == row)
      {
         stroke(255,0,0);
      }
      
      pushMatrix();
        // Scale up by 200S
        float factor = 200;
        translate(v.x * factor, v.y * factor, factor-v.z * factor);
        // Draw a point
        point(0,0);
      popMatrix();
    }
  }   
}

void saveRowsDepth(){
  filtLine1 = scans.get(0);
  filtLine2 = scans.get(1);
  filtLine3 = scans.get(2);
  
    for(int x = 0; x < kinect.width ; x += 1) {       
      filtLine1[x] = new PVector(x,row,depth[x+row*kinect.width]);
      filtLine2[x] = new PVector(x,(row+20), depth[x+(row+20)*kinect.width]);
      filtLine3[x] = new PVector(x,(row-20),depth[x+(row-20)*kinect.width]);
    }   
  
}


void findCorners(){  
  for(int I = 0; I < scans.size(); I++)
  {
    for(int x = 3*skipMes; x < scans.get(I).length-2*skipMes ; x++) 
    { 
      PVector v1 = depthToWorld(x, row, (int) (scans.get(I)[x]).z);
      
      if (v1.z <= 5)
      {      
        PVector m3 = depthToWorld((int) (scans.get(I)[x-3*skipMes]).x,(int) (scans.get(I)[x-3*skipMes]).y, (int) (scans.get(I)[x-3*skipMes]).z);
        PVector m1 = depthToWorld((int) (scans.get(I)[x-skipMes]).x, (int) (scans.get(I)[x-skipMes]).y, (int) (scans.get(I)[x-skipMes]).z);
        PVector p1 = depthToWorld((int) (scans.get(I)[x+skipMes]).x, (int) (scans.get(I)[x+skipMes]).y, (int)  (scans.get(I)[x+skipMes]).z);
        PVector p2 = depthToWorld((int) (scans.get(I)[x+2*skipMes]).x, (int) (scans.get(I)[x+2*skipMes]).y, (int)  (scans.get(I)[x+2*skipMes]).z);
        
        if (isCorner(m3.z,m1.z,v1.x,v1.y,v1.z,p1.z,p2.z))
        {
          // Corner Found
        }
      }
    }
  }
}

int detectDiscontinuities(float[] a)
{
  int cont = 0;
  for (int i = 1; i < a.length-1; i++)
  {
    if (a[i]-a[i-1]<-bigDist){
        listaCornerLeft.add(new CornerLeft(i,row, a[i]));
        cont++;
    }
    else if (a[i]-a[i-1]>bigDist){
        listaCornerRight.add(new CornerRight(i,row, a[i]));
         cont++;   
    }
  }  
  return cont; 
}

boolean isCorner(float m3,  float m1 , float px, float py, float pz, float p1, float p2)
{
  boolean cond = false;
  if ((p2-m3)<= -bigDist && (p1-m1)<= -bigDist && abs(p1-p2) <= smallDist /*&& abs(pz-m1) <= smallDist */ && (pz-m3) <= -bigDist)
  {
    stroke(255,0,255);
    listaCornerLeft.add(new CornerLeft(px,py,pz));
    cond =  true;
  }
  else if ((p2-m3) >= bigDist &&  (p1-m1) >= bigDist && abs(p1-p2) <= smallDist /* && abs(p1-p2) <= smallDist */ && abs(pz-m3) <= smallDist /*&& abs(pz-m3) <= smallDist */)
  {
    stroke(0,255,0);
    listaCornerRight.add(new CornerRight(px,py,pz)); 
    cond =  true;
  }    
  return cond;
}


PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

void drawMax()
{
  
}

void drawCorners(){
  strokeWeight(10);
  stroke(255,0,255);
  println("CL: " + listaCornerLeft.size()+ " CR: " + listaCornerRight.size());
  for (int i = 0; i < listaCornerLeft.size(); i++)
  {
    float x = (listaCornerLeft.get(i)).coordX;
    float y = (listaCornerLeft.get(i)).coordY;
    float z = (listaCornerLeft.get(i)).coordZ;
    pushMatrix();
      // Scale up by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
  stroke(0,255,0);
  for (int i = 1; i < listaCornerRight.size(); i++)
  {
    float x = (listaCornerRight.get(i)).coordX;
    float y = (listaCornerRight.get(i)).coordY;
    float z = (listaCornerRight.get(i)).coordZ;
    pushMatrix();
      // Scale up by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
}




boolean findMaximum(PVector[] a1){  
  boolean res = false;
  float far = 0;
  int xFar = 0;
  for(int x = skipMes*2; x < a1.length-skipMes*2 ; x++) 
  { 
    if (a1[x].z > far)
    {
      far =  a1[x].z;
      xFar = x;
    }
  }
  if (xFar > 0)  
  {
    maxOnLine = a1[xFar]; // depthToWorld(xFar, row, far);
    res = true;
    println("\t\t\t\t\t\tmax: " + maxOnLine.z);
  }
  return res; 
}

void keyPressed() {
   if (key == 'a') 
   {
    angle = angle + 1;
    kinect.setTilt(angle);
    println("UP");
  } 
  else if (key == 'b')
  {
    angle = angle - 1;
    kinect.setTilt(angle);
    println("DOWN");
  }
}