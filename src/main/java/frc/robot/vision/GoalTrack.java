package frc.robot.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.geometry.Pose2d;
import frc.robot.util.geometry.Rotation2d;
import java.util.Map;
import java.util.TreeMap;

/**
 * Used to keep track of all goals detected by vision system.
 *
 * <p>As goals are detected/not detected by vision system, function calls will be made to create,
 * destroy, or update a goal track. T As goals are detected/not detected by vision system, function
 * calls will be made to create, destroy, update goal track
 */
public class GoalTrack {
  TreeMap<Double, Pose2d> mObservedPositions = new TreeMap<>();

  // Smoothed Pose Position
  Pose2d mSmoothedPosition = null;

  public double VisionDistance;

  // Indentiferier of the goal track
  int mTrackID;

  /** Make a new Goal Track Stemmed from timestamp & goal's coodinerates */
  public static synchronized GoalTrack makeNewTrack(
      int id, double timestamp, Pose2d IntialGoalObservation) {
    GoalTrack gt = new GoalTrack();
    gt.mObservedPositions.put(timestamp, IntialGoalObservation);
    gt.mSmoothedPosition = IntialGoalObservation;
    gt.mTrackID = id;
    return gt;
  }

  // Empty Update
  // Called if we are not updating a track
  public void emptyUpdate() {
    PruneTracksByTime();
  }

  /**
   * Try to update the track with a new observation
   *
   * <p>Will return true if the track is succesfully updated!
   */
  public synchronized boolean tryUpdate(double timestamp, Pose2d observation) {
    boolean TargetAlive = IsGoalTrackAlive();
    if (TargetAlive == false) {
      return false;
    }

    // Get Distance to target
    double distance = mSmoothedPosition.inverse().transformBy(observation).getTranslation().norm();

    // if target is within our max tracking distance
    if (distance < Constants.kMaxTrackerDistance) {

      mObservedPositions.put(timestamp, observation);
      PruneTracksByTime();
      return true;
    } else {
      // outside of distance
      // perform an empty update
      emptyUpdate();
      return false;
    }
  }

  private GoalTrack() {}

  // Get the Tracks ID
  public synchronized int getID() {
    return mTrackID;
  }

  // Does this Goal Track contains observed positions
  public synchronized boolean IsGoalTrackAlive() {
    if (mObservedPositions.size() > 0) {
      return true;
    } else {
      return false;
    }
  }

  /** Removes the track if it is older than the set age this value is defined in constants file */
  private synchronized void PruneTracksByTime() {
    // Calculate deletion point..delete all before this time
    double delete_point = Timer.getFPGATimestamp() - Constants.kMaxGoalTrackAge;

    // Iterate through observed positions... removing if old
    mObservedPositions.entrySet().removeIf(entry -> entry.getKey() < delete_point);

    // Delete is now done!
    // Check if we are collecting anymore positions
    // if we are we should perform a track
    if (mObservedPositions.isEmpty() == true) {
      // No longer have a position
      mSmoothedPosition = null;
    } else {
      // Perform a smooth!
      SmoothObservations();
    }
  }

  /** Smooths positions Takes average of all tracked positions */
  private synchronized void SmoothObservations() {
    if (IsGoalTrackAlive() == true) {
      // if we have tracks
      // x position, y position, sin, cos
      double x = 0, y = 0, sin = 0, cos = 0;
      double vd = 0;
      double CurrentTime = Timer.getFPGATimestamp();
      double TimeDelta = 0;
      int ValueCount = 0;

      for (Map.Entry<Double, Pose2d> entry : mObservedPositions.entrySet()) {
        TimeDelta = CurrentTime - entry.getKey();
        if (TimeDelta > Constants.kMaxGoalTrackSmoothingTime) {
          continue;
        }

        // Track that we've seen another value
        ValueCount++;
        x += entry.getValue().getTranslation().x();
        y += entry.getValue().getTranslation().y();
        cos += entry.getValue().getRotation().cos();
        sin += entry.getValue().getRotation().sin();
        vd += entry.getValue().StoredDistance;
      }

      if (ValueCount == 0) {
        // if we found that all samples are older than the max smoothing time
        // just set our current position (smoothed position)
        mSmoothedPosition = mObservedPositions.lastEntry().getValue();
        VisionDistance = mSmoothedPosition.StoredDistance;
        int iii = 0;
      } else {
        // We have Samples
        // Average each Sample
        // Because we check that the goal track is alive.. we know at least 1 track
        x /= ValueCount;
        y /= ValueCount;
        sin /= ValueCount;
        cos /= ValueCount;

        // Create a new Pose2D object
        mSmoothedPosition = new Pose2d(x, y, new Rotation2d(cos, sin, true));
        VisionDistance = (vd / ValueCount);
        int iiii = 0;
      }
    }
  }

  // Get Smoothed Position (basically average)
  public synchronized Pose2d getSmoothedPosition() {
    return mSmoothedPosition;
  }

  // Get laast value we have observed
  public synchronized Pose2d getLatestPosition() {
    return mObservedPositions.lastEntry().getValue();
  }

  // Get last timestamp we have observed
  // return 0 if nothing
  public synchronized double getLatestTimestamp() {
    return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
  }

  public synchronized double getStability() {
    return Math.min(
        1.0, mObservedPositions.size() / (Constants.kCameraFrameRate * Constants.kMaxGoalTrackAge));
  }
}
