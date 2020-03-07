package frc.robot.vision;

/*
    Vision Packet back from RobotTracker.
    This is all the rest of Robot should care about
    Contains:
        Distance
        Error Angle
        Timestamp

*/

public class VisionPacket {

  public final double Distance;
  public final double Timestamp;
  public final double Error_Angle;
  public final boolean HasValue;
  public int ID = 0;

  private double Angle_Offset = 0;

  public VisionPacket(double TS, double Angle, double Dis) {
    Distance = Dis;
    Timestamp = TS;
    Error_Angle = Angle;
    HasValue = true;
  }

  public VisionPacket() {
    Distance = 0;
    Timestamp = 0;
    Error_Angle = 0;
    HasValue = false;
  }

  public void setID(int idarg) {
    ID = idarg;
  }

  public int getID() {
    return ID;
  }

  public void setTurretOffset(double angle){
    Angle_Offset = angle;
  }

  public double getTurretOffset(){
    return Math.toDegrees(Math.atan(.4 / Distance));
  }
}
