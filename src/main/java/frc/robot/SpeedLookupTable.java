package frc.robot;

import java.util.HashMap;

public class SpeedLookupTable {
  static SpeedLookupTable mInstance;
  public static synchronized SpeedLookupTable getInstance() {
    if (mInstance == null) {
      mInstance = new SpeedLookupTable();
    }
    return mInstance;
  }

  // the key is distance in meters
  // the value is velocity in whatever our system uses
  // note: This MUST be sorted by lowest distance first
  double[][] map = {
    {10d, 10d},
    {20d, 20d},
    {30d, 30d},
    {40d, 40d},
    {50d, 50d},
    {60d, 60d}
  };

  SpeedLookupTable(){
    
  }

  public double getSpeedFromDistance(double distance){
    double lowerBoundDistance = 0;
    double lowerBoundSpeed = 0;
    double upperBoundDistance = 0;
    double upperBoundSpeed = 0;
    for(int i = 0; i < map.length ; i++){
      double thisDistance = map[i][0];
      double thisSpeed = map[i][0];
      lowerBoundDistance = upperBoundDistance;
      lowerBoundSpeed = upperBoundSpeed;
      upperBoundDistance = thisDistance;
      upperBoundSpeed = thisSpeed;

      if(distance > lowerBoundDistance && distance < upperBoundDistance){
        return (lowerBoundSpeed + upperBoundSpeed) / 2;
      }
    }
    return upperBoundSpeed;
  }
}