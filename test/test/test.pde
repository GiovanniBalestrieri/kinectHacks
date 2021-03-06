import org.openkinect.processing.*;

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


void setup(){
  // Rendering in P3D
  size(800, 600, P3D);
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
}

void draw(){
  background(0);
  //angle = angle - 0.1;
  depth = kinect.getRawDepth();
  //kinect.setTilt(angle);
  
  
  // Translate and rotate
  translate(width/2, height/2, 300);
  stroke(255);
  beginShape(POINTS);
  strokeWeight(2);
  
  for(int x = 0; x < kinect.width ; x += skip) {
    for(int y = 0; y < kinect.height; y += skip) {
         
      int offset = x + y * kinect.width;


      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);

      if (y == row || y == row1)
      {
        if (x>skip && x < kinect.width - skip*1)
        {
          PVector m1 = depthToWorld(x-skipMes, y, depth[x-skipMes + y * kinect.width]);
          PVector p1 = depthToWorld(x+skipMes, y, depth[x+skipMes + y * kinect.width]);
          if (isCorner(m1.z,v.x,v.y,v.z,p1.z))
          {
            strokeWeight(10);
          }
          else
            stroke(255,0,0);
        }
        else
        {
          strokeWeight(2);
          stroke(255,0,0);
        }
      }
      else
      {
        strokeWeight(2);
        stroke(255);  
      }
      if (x == kinect.width/2 && y == kinect.height/2)
          println(v.z);
      
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

boolean isCorner(float m1, float px, float py, float p , float p1)
{
  boolean cond = false;
  if ((p1-m1)<= -bigDist /*&& abs(p-p1) <= smallDist */&& (p-m1)<= -bigDist*0.9)
  {
    stroke(255,0,255);
    listaCornerLeft.add(new CornerLeft(px,py));
    cond =  true;
  }
  else if ((p1-m1)>= bigDist && abs(p-m1) <= smallDist && (p-p1) <= -bigDist*0.9)
  {
    stroke(0,255,0);
    listaCornerRight.add(new CornerRight(px,py)); 
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