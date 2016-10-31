class CornerRight{
 public float coordX;
 public float coordY;
 public float coordZ;
 public float cornerDeep;
 public int type;
 
 CornerRight(float x, float y, float z, int t)
 {
   coordX = x;
   coordY = y;
   coordZ = z;
   type = t;
 }
 
 CornerRight(float x,float y, float z, float deep)
 {
   cornerDeep = deep;
   coordX = x;
   coordY = y;
   coordZ = z;
 }
}

class CornerLeft{
 public float coordX;
 public float coordY;
 public float coordZ;
 public float cornerDeep;
 public int type;
 
 CornerLeft(float x, float y, float z, int t)
 {
   coordX = x;
   coordY = y;
   coordZ = z;
   type = t;
 }
 
 CornerLeft(float x,float y , float z, float deep)
 {
   cornerDeep = deep;
   coordX = x;
   coordY = y;
 }
}