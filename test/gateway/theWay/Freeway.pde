public class Gateway
{
  int minX;
  int gateElements[] = new int[800];
  PVector vector;
  ArrayList<PVector> gateElem;
  
  public Gateway(int xmin, PVector vec)
  { 
    minX = xmin;
    gateElements[0] = xmin;
    gateElem = new ArrayList<PVector>();
    gateElem.add(vec);    
  }
  
  public Gateway()
  {   
    gateElem = new ArrayList<PVector>();  
  }
  
  public void addElem(int x, PVector vec)
  {
    gateElements = append(gateElements,x);
    gateElem.add(vec);
  }
  
  public int size()
  {
    gateElem.size();
    return gateElements.length;
  }
  
  public int middleValue()
  {
    int horizonPosition = gateElements[gateElements.length/2];
    return horizonPosition;
  }
  
  public PVector middlePoint()
  {
    return  gateElem.get(gateElem.size()/2);
  }
}