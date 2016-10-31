class ObjectNeg{
 public CornerLeft cl;
 public CornerRight cr;
 public PVector center;
 
 ObjectNeg(CornerLeft l, CornerRight r){
   cl = l;
   cr = r;
   center.x = (cr.coordX - cl.coordX)/2; 
   center.y = (cr.coordY - cl.coordY)/2; 
   center.z = (cr.coordZ - cl.coordZ)/2; 
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