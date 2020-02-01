package frc.robot;

public class SpeedLookupTable {
  private static SpeedLookupTable mInstance;

  public static synchronized SpeedLookupTable getInstance() {
    if (mInstance == null) {
      mInstance = new SpeedLookupTable();
    }
    return mInstance;
  }

  // note: This MUST be sorted by lowest distance first
  double[][] mLookupTable = {
    // {distance in meters, speed in whatever our system uses}
    {10d, 100d},
    {20d, 200d},
    {30d, 300d},
    {40d, 400d},
    {50d, 500d},
    {60d, 600d}
  };

  SpeedLookupTable() {}

  // https://en.wikipedia.org/wiki/Linear_interpolation
  public double linearInterpolate(double x, double x0, double y0, double x1, double y1) {
    return (y0 * (x1 - x) + y1 * (x - x0)) / (x1 - x0);
  }

  public double getSpeedFromDistance(double distance) {
    double lowerBoundDistance = 0;
    double lowerBoundSpeed = 0;
    double upperBoundDistance = 0;
    double upperBoundSpeed = 0;
    for (int i = 0; i < mLookupTable.length; i++) {
      double thisDistance = mLookupTable[i][0];
      double thisSpeed = mLookupTable[i][1];

      if (thisDistance == distance) {
        return thisSpeed;
      }

      lowerBoundDistance = upperBoundDistance;
      lowerBoundSpeed = upperBoundSpeed;
      upperBoundDistance = thisDistance;
      upperBoundSpeed = thisSpeed;

      if (distance > lowerBoundDistance && distance < upperBoundDistance) {
        return linearInterpolate(
            distance, lowerBoundDistance, lowerBoundSpeed, upperBoundDistance, upperBoundSpeed);
      }
    }

    // we ran out of keys to loop through
    return upperBoundSpeed;
  }
}
