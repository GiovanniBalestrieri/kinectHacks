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
float[] corners = new float[10];
MedianFilter medLine = new MedianFilter((byte)3,0);

int[] filtLine1;
int[] filtLine2;
int[] filtLine3;
int contSkip = 0, historyClean = 0;
int frameRateKinect = 12;

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
  println(" f "+ kinect.width + " ; " + kinect.height);
  filtLine1 = new int[(int) (kinect.width/skip)+1];
  filtLine2 = new int[(int) (kinect.width/skip)+1];
  filtLine3 = new int[(int) (kinect.width/skip)+1];
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
  //findCorners(filtLine1,filtLine2,filtLine3);  
  contSkip = 0;  
  drawScenePCL();
  updateCounters();
}

void updateCounters()
{
  historyClean ++;
}

void cleanCornersMemory(){
  if (historyClean == frameRateKinect)
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
      
      /*
      if (y == row && x >= 3*skipMes && x < kinect.width-2*skipMes)
      {
          if (v.z <= 5 )
          {
            PVector m3 = depthToWorld(x-3*skipMes, row, depth[offset - 3*skipMes]);
            PVector m1 = depthToWorld(x-skipMes, row, depth[offset - skipMes]);
            PVector p1 = depthToWorld(x+skipMes, row, depth[offset +skipMes]);
            PVector p2 = depthToWorld(x+2*skipMes, row, depth[offset +2*skipMes ]);
            if (isCorner(m3.z,m1.z,v.x,v.y,v.z,p1.z,p2.z))
            {
              // Corner Found
              strokeWeight(10);
            }
            else
            {
              stroke(255,0,0);
            }
          }
      }
      */
      
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
 //drawCorners();  
}
void saveRowsDepth(){
  for(int x = 0; x < kinect.width ; x += skip) { 
    //medLine.in((float) depth[x+row*kinect.width]);
    filtLine1[contSkip] = (int) depth[x+row*kinect.width];
    filtLine2[contSkip] = (int) depth[x+(row+1)*kinect.width];
    filtLine3[contSkip] = (int) depth[x+(row+2)*kinect.width];
    contSkip++;
  }   
}

void findCorners(int[] a1, int[] a2, int[] a3){  
  for(int x = 2*skipMes; x < a1.length-2*skipMes ; x++) 
  { 
    PVector v1 = depthToWorld(x, row, a1[x]);
    PVector v2 = depthToWorld(x, row, a2[x]);
    PVector v3 = depthToWorld(x, row, a3[x]);
    if (v1.z <= 5 && v2.z <= 5 && v3.z <= 5)
    {
      
      PVector m3 = depthToWorld(x-3*skipMes, row, a1[x-3*skipMes]);
      PVector m1 = depthToWorld(x-skipMes, row, a1[x-skipMes]);
      PVector p1 = depthToWorld(x+skipMes, row, a1[x+skipMes]);
      PVector p2 = depthToWorld(x+2*skipMes, row, a1[x+2*skipMes]);
      // Sistema sotto
      if (isCorner(m3.z,m1.z,v1.x,v1.y,v1.z,p1.z,p2.z))
      {
        // Corner Found
        //strokeWeight(10);
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
    listaCornerLeft.add(new CornerLeft(px,py,p1));
    cond =  true;
  }
  else if ((p2-m3) >= bigDist &&  (p1-m1) >= bigDist && abs(p1-p2) <= smallDist /* && abs(p1-p2) <= smallDist */ && abs(pz-m3) <= smallDist /*&& abs(pz-m3) <= smallDist */)
  {
    stroke(0,255,0);
    listaCornerRight.add(new CornerRight(px,py,m1)); 
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
  strokeWeight(5);
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