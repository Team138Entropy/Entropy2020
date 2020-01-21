package frc.robot.vision;


import frc.robot.util.geometry.Pose2d;

/**
 * Partially adapted from 254. Used in the event that multiple goals are detected to judge all goals
 * based on: timestamp, stability, continuation of previous goals (if a goal was detected earlier)
 * helps smooth out vibration from camera
 */
public class GoalTracker {

    public static class TrackReport {

        // Transform from the field frame to the vision target
        public Pose2d field_to_target;

        // timestamp of the lastest time that the goal has been observed
        public double latest_timestamp;

        // Percentage of the goal tracking time during which this goal has been observed
        // range from 0 -> 1

        // Track Report ID
        public int id;
    }
}
