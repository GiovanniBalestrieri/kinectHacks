class CornerRight{
 public float coordX;
 public float coordY;
 public float coordZ;
 public float cornerDeep;
 
 CornerRight(float x, float y, float z)
 {
   coordX = x;
   coordY = y;
   coordZ = z;
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
 
 CornerLeft(float x, float y, float z)
 {
   coordX = x;
   coordY = y;
   coordZ = z;
 }
 
 CornerLeft(float x,float y , float z, float deep)
 {
   cornerDeep = deep;
   coordX = x;
   coordY = y;
 }
}