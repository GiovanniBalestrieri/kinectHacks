class Corner{
 public float coordX;
 public float coordY;
 public float cornerDeep;
 
 Corner(float x, float y)
 {
   coordX = x;
   coordY = y;
 }
 
 Corner(float x,float y, float deep)
 {
   cornerDeep = deep;
   coordX = x;
   coordY = y;
 }
}