package frc.robot.vision;

import frc.robot.util.geometry.Pose2d;
import java.util.TreeMap;

/**
 * Used to keep track of all goals detected by vision system.
 *
 * <p>As goals are detected/not detected by vision system, function calls will be made to create,
 * destroy, update goal track
 */
public class GoalTrack {
    TreeMap<Double, Pose2d> mObservedPositions = new TreeMap<>();

    Pose2d mSmoothedPosition = null;

    // Indentiferier of the goal track
    int mTrackID;

    /** Make a new Goal Track Stemmed from timestamp & goal's coodinerates */
    /*
    public static synchronized GoalTrack makeNewTrack(int id, double timestamp, Pose2d GoalPose){
        GoalTrack gt = new GoalTrack();
    }
    */

}
