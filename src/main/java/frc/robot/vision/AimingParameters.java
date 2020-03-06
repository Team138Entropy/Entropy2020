package frc.robot.vision;

import frc.robot.util.geometry.*;

/*
  Aiming Parameter

  Packed up information for the robot to use
*/
public class AimingParameters {
  private final double range;
  private final Pose2d robot_to_goal;
  private final Pose2d field_to_goal;
  private final Rotation2d robot_to_goal_rotation;
  private final double last_seen_timestamp;
  private final double stability;
  private final Rotation2d field_to_vision_target_normal;
  private final int track_id;

  private double VisionDistance;

  public AimingParameters(
      int trackIDarg,
      double last_seen_timestamparg,
      double stabilityarg,
      Pose2d robot_to_goalarg,
      Pose2d field_to_goalarg,
      Rotation2d field_to_visionarg) {
    this.track_id = trackIDarg;
    this.last_seen_timestamp = last_seen_timestamparg;
    this.stability = stabilityarg;
    this.robot_to_goal = robot_to_goalarg;
    this.field_to_goal = field_to_goalarg;
    this.field_to_vision_target_normal = field_to_visionarg;
    this.range = robot_to_goalarg.getTranslation().norm();
    this.robot_to_goal_rotation = robot_to_goalarg.getTranslation().direction();
  }

  public void SetVisionRange(double range) {
    VisionDistance = range;
  }

  public double GetVisionRange() {
    return VisionDistance;
  }

  // Robots pose to the goal
  public Pose2d getRobotToGoal() {
    return robot_to_goal;
  }

  public Pose2d getFieldToGoal() {
    return field_to_goal;
  }

  public double getRange() {
    return range;
  }

  public double getDistance() {
    return range;
  }

  public Rotation2d getRobotToGoalRotation() {
    return robot_to_goal_rotation;
  }

  public double getLastSeenTimestamp() {
    return last_seen_timestamp;
  }

  public double getStability() {
    return stability;
  }

  public Rotation2d getFieldToVisionTargetNormal() {
    return field_to_vision_target_normal;
  }

  public int getTrackID() {
    return track_id;
  }

  public int getID() {
    return track_id;
  }
}
