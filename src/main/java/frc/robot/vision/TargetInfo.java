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
 *      next define an imaginary view plane and compute it size
 *      for simplicity, we place this plane 1 unit (1.0) in front of camera
 * 
 *      view plane width = vpw = 2.0*tan(horizontal_fov/2)
 *      view plane height = vph = 2.0*tan(vertical_fov/2)
 * 
 *      using these two values, we can now convert between normalized pixel coordinates
 * 
 *      x = vpw/2 * nx
 *      y = vph/2 * ny
 * 
 *      ax = horizontal angle (x angle)
 *      ay = vertical angle
 *      tan(ax) = x/1
 *      tan(ay) = y/1
 *      ax = atan2(1,x)
 *      ay = atan2(1,y)d
 *      in 254's case:
 *          double nY = -((y_pixels - 160.0) / 160.0);
            double nZ = -((z_pixels - 120.0) / 120.0);
            double y = Constants.kVPW / 2 * nY;
            double z = Constants.kVPH / 2 * nZ;

            then they use the y and z values as y and z
            x just represents 1 unit in front of you!
            z is height, y is left -> right

            (0,0,0) is the point on the floor immediately underneath the robots center of rattation

            in their case, their lightlight was -24 degrees angled down

        xz plane translation:
            the xz translation 
 * 
 * 
 */
public class TargetInfo {

    protected double x = 1.0;
    protected double y;
    protected double z;
    private double yaw = 0;
    private int CameraID = 0;

    private int TargetType = 0;

    // Distance in Feet Currently
    private double distance = 0;

    public TargetInfo() {}

    public TargetInfo(int TargType, double y, double z, double distance){
        this.TargetType = TargType;
        this.y = y;
        this.z = z;
        this.distance = distance;
    }

    public TargetInfo(int CameraID, int TargetType, int x, int y, int z, double distance) {
        this.CameraID = CameraID;
        this.TargetType = TargetType;
        this.x = 1.0;
        this.y = y;
        this.z = z;
        this.distance = distance;
    }

    /*
        Convert Fields to Limelights vision concept
        0,0 is top left most corner
        
        

    */
    public void CalculateFields(){
        //
        switch(TargetType){
            case 0:
                //packets from High Goal Camera


            break;
            case 1:
                //packets from Ball Tracking Camera


            break;
        }
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
