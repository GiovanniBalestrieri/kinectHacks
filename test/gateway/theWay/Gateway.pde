class Gateway{
 public PVector center;
 
 Gateway(){
 center = new PVector(0,0,0);
 }
 
 Gateway(PVector c){
   center = new PVector(c.x,c.y,c.z);
 }
  
}