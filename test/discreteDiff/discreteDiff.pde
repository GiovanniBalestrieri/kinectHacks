import org.openkinect.processing.*;
import peasy.*;


PeasyCam cam;

Kinect kinect;




PImage img;
int[] depth;
float angle;
int skipX,skipY;
int skipMes = 14;
// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];
float bigDist = 0.30f, smallDist = 0.15f;
int row = 350, row1 = 370;

HashMap<CornerLeft,CornerLeft> distanzeL = new HashMap<CornerLeft,CornerLeft>();
HashMap<CornerRight,CornerRight> distanzeR = new HashMap<CornerRight,CornerRight>();

float neighborDist = 0.1;

int scanUp = 5, scanDown = 5;

ArrayList<CornerLeft> listaCornerLeftUp = new ArrayList<CornerLeft>();
ArrayList<CornerRight> listaCornerRightUp = new ArrayList<CornerRight>();

ArrayList<CornerLeft> listaCornerLeftDown = new ArrayList<CornerLeft>();
ArrayList<CornerRight> listaCornerRightDown = new ArrayList<CornerRight>();

ArrayList<CornerLeft> listaCornerLeftMid = new ArrayList<CornerLeft>();
ArrayList<CornerRight> listaCornerRightMid = new ArrayList<CornerRight>();
ArrayList<ObjectNeg> objects = new ArrayList<ObjectNeg>();
ArrayList<Gateway> gateways = new ArrayList<Gateway>();


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
  
  println(showGatewayPoint);
  
  frameRate(frameRateKinect);
  
  cam = new PeasyCam(this, 100);
  cam.setMinimumDistance(0);
  cam.setMaximumDistance(1500);
  cam.setSuppressRollRotationMode();
  
  kinect = new Kinect(this);
  kinect.initDepth();
  skipY = 7; skipX=7;
  angle = kinect.getTilt();
 
  depthLookUp = new float[2048];
  for (int i = 0; i < depthLookUp.length; i++) 
  {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  
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
  
  detectObjects();
  
  updateCounters();
}

// TODO fix it
void detectObjects(){
  for (int i=0;i<listaCornerRightUp.size();i++){
    boolean cond = false;
    for (int j=0;j<listaCornerLeftUp.size();j++){
      
      println("AA  R" + (listaCornerRightUp.get(i)).coordX + " L: " + (listaCornerLeftUp.get(j)).coordX);
      // Se si trova più a dx del corner in esame, salva dist
      if ((listaCornerRightUp.get(i)).coordX > (listaCornerLeftUp.get(j)).coordX){
        
         println("BB  R" + (listaCornerRightUp.get(i)).coordX + " L: " + (listaCornerLeftUp.get(j)).coordX);
        float a = Norm((listaCornerRightUp.get(i)).coordX,(listaCornerLeftUp.get(j)).coordX);
        
        Gateway g = new Gateway((listaCornerRightUp.get(i)).coordX+a/2,(listaCornerRightUp.get(i)).coordY,(listaCornerRightUp.get(i)).coordZ);
        gateways.add(g);
        
        stroke(0,255,255);
        strokeWeight(15);
        pushMatrix();
          // Scale up by 200S
          float factor = 200;
          translate(g.center.x * factor, g.center.y * factor, factor - g.center.z * factor);
          // Draw a point
          point(0,0);
        popMatrix();
        
        cond = true;
        
        break;
        /*
        if (a < neighborDist){
                distanzeL.put(listaCornerLeftUp.get(j),listaCornerRightUp.get(i));
        }
        */
      }
      if (cond)
      {
        cond = false;
        break;
      }
      
    }
    /*
    boolean cond = false;
    for (int j=0;j<listaCornerRightUp.size() && !cond;j++){
      if ( !cond && listaCornerLeftUp.get(i).coordX < listaCornerRightUp.get(j).coordX){
       cond = true; 
       ObjectNeg obj = new ObjectNeg(listaCornerLeftUp.get(i),listaCornerRightUp.get(j));
         
        stroke(0,255,255);
        strokeWeight(15);
        pushMatrix();
          // Scale up by 200S
          float factor = 200;
          translate(listaCornerLeftUp.get(i).coordX * factor, listaCornerLeftUp.get(i).coordY * factor, factor-  listaCornerLeftUp.get(i).coordZ * factor);
          // Draw a point
          point(0,0);
        popMatrix();
        
        stroke(255,255,0);
        strokeWeight(15);
        pushMatrix();
          // Scale up by 200S
          translate(listaCornerRightUp.get(j).coordX * factor, listaCornerRightUp.get(j).coordY * factor, factor-  listaCornerRightUp.get(j).coordZ * factor);
          // Draw a point
          point(0,0);
        popMatrix();
        
        println(" idm : " + listaCornerLeftUp.get(i).coordX + "\t" + listaCornerRightUp.get(j).coordX  );
           
        stroke(0,0,255);
        strokeWeight(20);
        pushMatrix();
          // Scale up by 200S
          translate(obj.center.x * factor, obj.center.y * factor, factor-  listaCornerRightUp.get(j).coordZ * factor);
          // Draw a point
          point(0,0);
        popMatrix();
      
      }
    }
    */
  }
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
      for (int i = listaCornerLeftUp.size() - 1; i >= 0; i--) {
        listaCornerLeftUp.remove(i);
      } 
      for (int i = listaCornerRightUp.size() - 1; i >= 0; i--) {
        listaCornerRightUp.remove(i);
      } 
      
      
      for (int i = listaCornerLeftMid.size() - 1; i >= 0; i--) {
        listaCornerLeftMid.remove(i);
      } 
      for (int i = listaCornerRightMid.size() - 1; i >= 0; i--) {
        listaCornerRightMid.remove(i);
      } 
      
      
      for (int i = listaCornerLeftDown.size() - 1; i >= 0; i--) {
        listaCornerLeftDown.remove(i);
      } 
      for (int i = listaCornerRightDown.size() - 1; i >= 0; i--) {
        listaCornerRightDown.remove(i);
      } 
      historyClean = 0;
  }
}

void drawScenePCL(){  
 //println("UPP\t\tCL: " + listaCornerLeftUp.size()+ " CR: " + listaCornerRightUp.size());
  
 for(int x = 0; x < kinect.width ; x += skipX) {   
    for(int y = 0; y < kinect.height; y += skipY) {         
      int offset = x + y * kinect.width;
      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      
      strokeWeight(2);
      stroke(255);  
      
      if (y == row || y == (row+scanUp*skipY) || y == (row-scanDown*skipY))
      {
         strokeWeight(4);
         stroke(255,0,0);
      }
      
      /*
      for (int b = 0; b<listaCornerRightUp.size();b++)
      {
        // use euclidean norm instead
        if (x-listaCornerRightUp.get(b).coordX <= 10 && y-listaCornerRightUp.get(b).coordY <= 10 && v.z-listaCornerRightUp.get(b).coordX <= smallDist)
        {
          println("AAA");
           stroke(255,0,0);
        }
      }
      */
      
      
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

void detectObjectDepth()
{
  for (int i = 0; i < listaCornerLeftMid.size(); i++)
  {
      // (listaCornerLeftMid.get(i)).coordX;
  }
}

void saveRowsDepth(){
  filtLine1 = scans.get(0);
  filtLine2 = scans.get(1);
  filtLine3 = scans.get(2);
  
    for(int x = 0; x < kinect.width ; x += 1) {       
      filtLine1[x] = new PVector(x,(row+5*scanUp), depth[x+(row+5*scanUp)*kinect.width]);
      filtLine2[x] = new PVector(x,row,depth[x+row*kinect.width]);
      filtLine3[x] = new PVector(x,(row-5*scanDown),depth[x+(row-5*scanDown)*kinect.width]);
    }     
}


void findCorners(){  
  for(int I = 0; I < scans.size(); I++)
  {
    for(int x = 3*skipMes; x < scans.get(I).length-2*skipMes ; x++) 
    { 
      PVector v1 = depthToWorld((int) (scans.get(I)[x]).x, (int) (scans.get(I)[x]).y, (int) (scans.get(I)[x]).z);
      
      if (v1.z <= 5)
      {      
        PVector m3 = depthToWorld((int) (scans.get(I)[x-3*skipMes]).x,(int) (scans.get(I)[x-3*skipMes]).y, (int) (scans.get(I)[x-3*skipMes]).z);
        PVector m1 = depthToWorld((int) (scans.get(I)[x-skipMes]).x, (int) (scans.get(I)[x-skipMes]).y, (int) (scans.get(I)[x-skipMes]).z);
        PVector p1 = depthToWorld((int) (scans.get(I)[x+skipMes]).x, (int) (scans.get(I)[x+skipMes]).y, (int)  (scans.get(I)[x+skipMes]).z);
        PVector p2 = depthToWorld((int) (scans.get(I)[x+2*skipMes]).x, (int) (scans.get(I)[x+2*skipMes]).y, (int)  (scans.get(I)[x+2*skipMes]).z);
        
        if (isCorner(m3.z,m1.z,v1.x,v1.y,v1.z,p1.z,p2.z,I))
        {
          
        }
      }
    }
  }
}

boolean isCorner(float m3,  float m1 , float px, float py, float pz, float p1, float p2,int i)
{
  boolean cond = false;
  if ((p2-m3)<= -bigDist && (p1-m1)<= -bigDist && abs(p1-p2) <= smallDist /*&& abs(pz-m1) <= smallDist */ && (pz-m3) <= -bigDist)
  {
    stroke(255,0,255);
    if (i == 0)
      listaCornerLeftUp.add(new CornerLeft(px,py,pz,i));
    if (i == 1)
      listaCornerLeftMid.add(new CornerLeft(px,py,pz,i));
    if (i == 2)
      listaCornerLeftDown.add(new CornerLeft(px,py,pz,i));
    cond =  true;
  }
  else if ((p2-m3) >= bigDist &&  (p1-m1) >= bigDist && abs(p1-p2) <= smallDist /* && abs(p1-p2) <= smallDist */ && abs(pz-m3) <= smallDist /*&& abs(pz-m3) <= smallDist */)
  {
    stroke(0,255,0);
    if (i == 0)
      listaCornerRightUp.add(new CornerRight(px,py,pz,i)); 
    if (i == 1)
      listaCornerRightMid.add(new CornerRight(px,py,pz,i)); 
    if (i == 2)
      listaCornerRightDown.add(new CornerRight(px,py,pz,i)); 
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


void drawCorners(){
  strokeWeight(10);
  stroke(255,0,255);
  //println("UP \t\tCL: " + listaCornerLeftUp.size()+ " CR: " + listaCornerRightUp.size());
  for (int i = 0; i < listaCornerLeftUp.size(); i++)
  {
    float x = (listaCornerLeftUp.get(i)).coordX;
    float y = (listaCornerLeftUp.get(i)).coordY;
    float z = (listaCornerLeftUp.get(i)).coordZ;
    pushMatrix();
      // Scale up by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
  stroke(0,255,0);
  for (int i = 1; i < listaCornerRightUp.size(); i++)
  {
    float x = (listaCornerRightUp.get(i)).coordX;
    float y = (listaCornerRightUp.get(i)).coordY;
    float z = (listaCornerRightUp.get(i)).coordZ;
    pushMatrix();
      // Scale up by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
  
  // MID
  
  
  stroke(255,0,255);
  //println("Mid \t\tCL: " + listaCornerLeftMid.size()+ " CR: " + listaCornerRightMid.size());
  for (int i = 0; i < listaCornerLeftMid.size(); i++)
  {
    float x = (listaCornerLeftMid.get(i)).coordX;
    float y = (listaCornerLeftMid.get(i)).coordY;
    float z = (listaCornerLeftMid.get(i)).coordZ;
    pushMatrix();
      // Scale Mid by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
  stroke(0,255,0);
  for (int i = 1; i < listaCornerRightMid.size(); i++)
  {
    float x = (listaCornerRightMid.get(i)).coordX;
    float y = (listaCornerRightMid.get(i)).coordY;
    float z = (listaCornerRightMid.get(i)).coordZ;
    pushMatrix();
      // Scale Mid by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
  
  // DOWN
  
  stroke(255,0,255);
  //println("Down \t\tCL: " + listaCornerLeftDown.size()+ " CR: " + listaCornerRightDown.size());
  for (int i = 0; i < listaCornerLeftDown.size(); i++)
  {
    float x = (listaCornerLeftDown.get(i)).coordX;
    float y = (listaCornerLeftDown.get(i)).coordY;
    float z = (listaCornerLeftDown.get(i)).coordZ;
    pushMatrix();
      // Scale Down by 200S
      float factor = 200;
      translate(x * factor, y * factor, factor-z * factor);
      // Draw a point
      point(0,0);
    popMatrix();
  }
  stroke(0,255,0);
  for (int i = 1; i < listaCornerRightDown.size(); i++)
  {
    float x = (listaCornerRightDown.get(i)).coordX;
    float y = (listaCornerRightDown.get(i)).coordY;
    float z = (listaCornerRightDown.get(i)).coordZ;
    pushMatrix();
      // Scale Down by 200S
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

float  Norm(float a, float b){
  return sqrt(pow(a,2)+pow(b,2));
}