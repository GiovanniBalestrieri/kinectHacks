class MedianFilter {
  private float[] sortedData;    // array pointer for data sorted by size
  private byte[] historyMap;    // array pointer for locations of history data in sorted list
  private byte[] locationMap; 
  private byte medFilterWin;
  private byte medDataPointer; 
  int ODP;        // oldest data point in accelRawHistory
  float tempData;      // temp data storage while swapping data locations
  byte tempMap;      // temp map storage while swapping data locations
  boolean dataMoved;
  

  MedianFilter(byte size, int seed) {

    if (size < 3) {
      size = 3;
    }  //  prevent undersized windows

    medFilterWin = 3;        // number of samples in sliding median filter window - usually odd #
    medDataPointer = 2;      // mid point of window
    sortedData = new float[size];
    historyMap = new byte[size];
    locationMap = new byte[size];
    ODP = 0;              // oldest data point location in historyMap

    for (byte i=0; i<medFilterWin; i++) // initialize the arrays
    {
      historyMap[i] = i;        // start map with straight run
      locationMap[i] = i;        // start map with straight run
      sortedData[i] = seed;      // populate with seed value
    }
  }

  public float in(float value)
  {  
    sortedData[historyMap[ODP]] = value;  // store new data in location of oldest data

    dataMoved = false;

    if (historyMap[ODP] != 0) // don't check left neighbours if at the extreme left
    {
      for (int i=historyMap[ODP]; i>0; i--)  //index through left adjacent data
      {
        int j = i - 1;  // neighbour location
        if (sortedData[i] < sortedData[j])
        {
          //Serial.print("<");
          tempData = sortedData[j];    // store neighbour data in temp
          tempMap = locationMap[j];    // store position of adjacent data in historyMap

          sortedData[j] = sortedData[i];  // move new data to neighbour location
          historyMap[ODP] = (byte) j;
          locationMap[j] = (byte) ODP;

          sortedData[i] = tempData;    // swap neighbour data back in
          historyMap[tempMap] = (byte) i;
          locationMap[i] = tempMap;

          dataMoved = true;
        } else
        {
          i=0; // abort loop if left neighbour is larger (faster than "break;")
        }
      }
    } // end shift data to left

    if (historyMap[ODP] != medFilterWin - 1 && dataMoved == false) // don't check right neighbours if at the extreme right or data already moved
    {
      for (int i=historyMap[ODP]; i<medFilterWin-1; i++) //index through right adjacent data
      {
        int j = i + 1;  // neighbour location
        if (sortedData[i] > sortedData[j])
        {
          //Serial.print(">");
          tempData = sortedData[j]; // store neighbour data in temp
          tempMap = locationMap[j];  // store position of adjacent data in historyMap

          sortedData[j] = sortedData[i];// move new data to neighbour location
          historyMap[ODP] = (byte) j;
          locationMap[j] = (byte) ODP;

          sortedData[i] = tempData; // swap neighbour data back in
          historyMap[tempMap] = (byte) i;
          locationMap[i] = tempMap;
        } else
        {
          i=medFilterWin; // abort loop if right neighbour is smaller (faster than "break;")
        }
      }
    } // end shift data to right

    ODP = (ODP + 1) % medFilterWin; // increment and wrap

    return sortedData[medDataPointer];
  }


  public float out() // return the value of the median data sample
  {
    return  sortedData[medDataPointer];
  }


  public void printData() // display sorting data for debugging
  {
    for (int i=0; i<medFilterWin; i++)
    {
      print(sortedData[i]);
      print("\t");
    }
    print("\t");
    println("Data sorted by size");

    for (int i=0; i<medFilterWin; i++)
    {
      print(historyMap[i]);
      print("\t");
    }
    print("\t");
    println("Locations of data sorted by age");

    for (int i=0; i<medFilterWin; i++)
    {
      print(locationMap[i]);
      print("\t");
    }
    print("\t");
    println("Location of age data in age list sorted by data size");
    println("");
  }
}