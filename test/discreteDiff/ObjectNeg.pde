class ObjectNeg{
 public CornerLeft cl;
 public CornerRight cr;
 public PVector center;
 
 ObjectNeg(CornerLeft l, CornerRight r){
   cl = l;
   cr = r;
   float x = (r.coordX - l.coordX)/2; 
   float y = l.coordY;
   float z = (l.coordZ - r.coordZ)/2; 
   center = new PVector(x,y,z);
 }
 
 public void setCornerLeft(CornerLeft l)
 {
   cl = l;
 }
 
 
 public void setCornerRight(CornerRight r)
 {
   cr = r;
 } 
 
 
  
}