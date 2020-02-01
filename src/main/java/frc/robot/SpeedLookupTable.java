package frc.robot;


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
    for (int i = 0; i < map.length; i++) {
      double thisDistance = map[i][0];
      double thisSpeed = map[i][1];

      if (thisDistance == distance) {
        return thisSpeed;
      }

      lowerBoundDistance = upperBoundDistance;
      lowerBoundSpeed = upperBoundSpeed;
      upperBoundDistance = thisDistance;
      upperBoundSpeed = thisSpeed;

      System.out.println(
          " lowerBoundDistance: "
              + lowerBoundDistance
              + " lowerBoundSpeed: "
              + lowerBoundSpeed
              + " upperBoundDistance: "
              + upperBoundDistance
              + " upperBoundSpeed: "
              + upperBoundSpeed);

      if (distance > lowerBoundDistance && distance < upperBoundDistance) {
        return linearInterpolate(
            distance, lowerBoundDistance, lowerBoundSpeed, upperBoundDistance, upperBoundSpeed);
      }
    }

    // we ran out of keys to loop through
    return upperBoundSpeed;
  }
}
