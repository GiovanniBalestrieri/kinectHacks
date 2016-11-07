class Gateway{
 public PVector center;
 
 Gateway(){}
 
 Gateway(float x,float y,float z){
   center = new PVector(x,y,z);
 }
 
 
 Gateway(PVector c){
   center = c;
 }
  
}