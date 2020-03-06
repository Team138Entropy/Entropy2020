package frc.robot.subsystems;

import frc.robot.Kinematics;
import frc.robot.RobotTracker;
import frc.robot.util.geometry.*;
import frc.robot.util.loops.*;

/*
    Class that periodically updates the RobotTracker Object
    Feeds back information about the turret, drive system


*/
public class RobotTrackerUpdater extends Subsystem {

  private static RobotTrackerUpdater mInstance;

  private RobotTracker mRobotTracker = RobotTracker.getInstance();
  private Drive mDrive = Drive.getInstance();

  private double mLeft_encoder_prev_distance_ = 0.0;
  private double mRight_encoder_prev_distance_ = 0.0;
  private double mPrev_timestamp_ = -1.0;
  private Rotation2d mPrev_Rotation = null;

  public static synchronized RobotTrackerUpdater getInstance() {
    if (mInstance == null) {
      mInstance = new RobotTrackerUpdater();
    }
    return mInstance;
  }

  private RobotTrackerUpdater() {}

  @Override
  public void registerEnabledLoops(ILooper looper) {
    looper.register(new EnabledLoop());
  }

  private class EnabledLoop implements Loop {

    @Override
    public synchronized void onStart(double timestamp) {
      // Update Encoder Distance
      mLeft_encoder_prev_distance_ = mDrive.getLeftEncoderDistance();
      mRight_encoder_prev_distance_ = mDrive.getRightEncoderDistance();
      mPrev_timestamp_ = timestamp;
    }

    @Override
    public synchronized void onLoop(double timestamp) {
      // Intial Heading Update

      if (mPrev_Rotation == null) {
        // get robot's latest position of the field
        mPrev_Rotation = mRobotTracker.getLatestFieldToRobot().getValue().getRotation();
      }

      final double dt = timestamp - mPrev_timestamp_;
      final double left_distance = mDrive.getLeftEncoderDistance();
      final double right_distance = mDrive.getRightEncoderDistance();
      final double delta_left = left_distance - mLeft_encoder_prev_distance_;
      final double delta_right = right_distance - mRight_encoder_prev_distance_;

      // get gyros rotation!
      final Rotation2d gyro_angle = mDrive.getHeading();

      Twist2d odometry_twist;

      // Use Robot State as a lock
      synchronized (mRobotTracker) {
        final Pose2d last_measurement = mRobotTracker.getLatestFieldToRobot().getValue();
        odometry_twist =
            Kinematics.forwardKinematics(
                last_measurement.getRotation(), delta_left, delta_right, gyro_angle);
      }
      final Twist2d measured_velocity =
          Kinematics.forwardKinematics(
                  delta_left,
                  delta_right,
                  mPrev_Rotation.inverse().rotateBy(gyro_angle).getRadians())
              .scaled(1.0 / dt);
      final Twist2d predicted_velocity =
          Kinematics.forwardKinematics(
                  mDrive.getLeftLinearVelocity(), mDrive.getRightLinearVelocity())
              .scaled(dt);

      // add turrets current rotation
      // mRobotTracker.addVehicleToTurretObservation(timestamp,
      // Rotation2d.fromDegrees(Turret.getInstance().getAngle()));
      mRobotTracker.addDriveObservations(
          timestamp, odometry_twist, measured_velocity, predicted_velocity);
      mLeft_encoder_prev_distance_ = left_distance;
      mRight_encoder_prev_distance_ = right_distance;
      mPrev_Rotation = gyro_angle;
      mPrev_timestamp_ = timestamp;
    }

    @Override
    public void onStop(double timestamp) {}
  }

  @Override
  public void stopSubsytem() {}

  public void zeroSensors() {}

  public void checkSubsystem() {}
}
