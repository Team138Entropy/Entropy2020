package frc.robot.vision;

/**
 * Container Class for Targets detected by the division system Contains a center point in 3D Space
 *
 * <p>On the field there are 3 Types of Targets, 0 - High Goal 1 - Feeder Station 2 - Ball
 * 
 *  The vector has an x component (+x is out towards the goal) that is always set to 1; a y component (+y is to the left in the camera image); and a z component (+z is up).
 * 
 * 
 *  Last year 254 tracked the corners of the target.
 * 
 *  Y = Horizontal 
 *  Z = Vertical
 *  X = Forward (set to 1).. 1 unit away
 * 
 * 
 *  Limelight -> Pixels to Angles
 *      The limelight has a horizontal field of view of 54 degrees
 *      the limelight has a vertical field of view of 41 degrees
 *      Captures at a resolution of 320x240
 *      Assume center of camera is center of optical axis
 *              at this point the x and y angles are also zero
 *      using trig we can calculate 
 * 
 *      Step 1 -> Convert from pixel coordinates to normalized 2D coordinates
 *                  where 0,0 is the center of the image and 1.0
 * 
 *      (px, py) = pixel coords, 0,0 is upper left.. positive down and to the right
 *                  far right down is the positive directiona
 * 
 * 
 *      (nx, ny) = normalized pixel coords, 0,0 is the center, psoition right and up
 *                  this runs like a normal graph.. 
 * 
 *      nx = (1/160) * (px - 159.5)
 *      ny = (1/120) * (119.5 - py)     
 *          
 *             
 * 
 */
public class TargetInfo {

    protected double x = 1.0;
    protected double y = 0;
    protected double z = 1.0;
    private double yaw = 0;
    private int CameraID = 0;

    private int TargetType = 0;

    // Distance in Feet Currently
    private double distance = 0;

    public TargetInfo() {}

    public TargetInfo(int CameraID, int TargetType, int x, int y, int z, double distance) {
        this.CameraID = CameraID;
        this.TargetType = TargetType;
        this.x = 1.0;
        this.y = y;
        this.z = z;
        this.distance = distance;
    }

    public double getYaw(){
        return yaw;
    }


    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public void SetBallTarget() {
        TargetType = 2;
    }

    public void SetFeederStationTarget() {
        TargetType = 1;
    }

    public void SetHighgoalTarget() {
        TargetType = 0;
    }

    public void SetDistance(double dis) {
        this.distance = dis;
    }

    public void SetX(double x) {
        this.x = x;
    }

    public void SetY(double y) {
        this.y = y;
    }

    public void SetZ(double z){
        this.z = z;
    }

    public void SetTargetID(int id) {
        this.TargetType = id;
    }

    public void SetCameraID(int id) {
        this.CameraID = id;
    }

    public void SetYaw(double yaw) {
        this.yaw = yaw;
    }

    // Returns if we this target is a high goal target
    public boolean IsHighGoal() {
        if (TargetType == 0) {
            return true;
        }
        return false;
    }

    public boolean IsBall(){
        if(TargetType == 1){
            return true;
        }
        return false;
    }
}
