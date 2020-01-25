package frc.robot;

/*
    Robote state tracks the robot throughout the match


*/
public class RobotState {
  private static RobotState mInstance;

  Logger mRobotLogger = new Logger("robot");

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }

    return mInstance;
  }

  private RobotState() {}

  // Reset Robot State
  // Make note of the Starting Position
  public void reset() {
    mRobotLogger.log("Robot State Reset");
  }

  /*
      Record robot's initial position on field
      This is encapsulated in a Pose2D object
  */
  private void SetIntialFieldToRobot() {}
}
