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
  double[][] mPracticeLookupTable = {
    // {distance in meters, speed in whatever our system uses}
    {10d, 2600d},
    {20d, 2600d},
    {30d, 2600d},
    {40d, 2600d}
  };

  double[][] mProductionLookupTable = {
    // {distance in meters, speed in whatever our system uses}
    {10d, 2950},
    {20d, 2950},
    {30d, 2950},
    {40d, 2950}
  };

  SpeedLookupTable() {}

  // https://en.wikipedia.org/wiki/Linear_interpolation
  public double linearInterpolate(double x, double x0, double y0, double x1, double y1) {
    return (y0 * (x1 - x) + y1 * (x - x0)) / (x1 - x0);
  }

  public double getSpeedFromDistance(double distance) {
    double[][] ourTable;

    if (Robot.getIsPracticeBot()) {
      ourTable = mPracticeLookupTable;
    } else {
      ourTable = mProductionLookupTable;
    }

    double lowerBoundDistance = 0;
    double lowerBoundSpeed = 0;
    double upperBoundDistance = 0;
    double upperBoundSpeed = 0;
    for (int i = 0; i < ourTable.length; i++) {
      double thisDistance = ourTable[i][0];
      double thisSpeed = ourTable[i][1];

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
