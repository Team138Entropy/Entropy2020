package frc.robot.vision;

import frc.robot.Constants;
import frc.robot.util.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Partially adapted from 254. Used in the event that multiple goals are detected to judge all goals
 * based on: timestamp, stability, continuation of previous goals (if a goal was detected earlier)
 * helps smooth out vibration from camera
 */
public class GoalTracker {

  public static class TrackReport {

    // Transform from the field frame to the vision target
    public Pose2d field_to_target;

    public double distance;

    // timestamp of the lastest time that the goal has been observed
    public double latest_timestamp;

    // Percentage of the goal tracking time during which this goal has been observed
    // range from 0 -> 1
    public double stability;

    // Track Report ID
    public int id;

    public TrackReport(GoalTrack track) {
      // Inflate a TrackReport from a Goal Track
      field_to_target = track.getSmoothedPosition();
      latest_timestamp = track.getLatestTimestamp();
      stability = track.getStability();
      id = track.getID();
      distance = track.VisionDistance;
    }
  }

  /**
   * TrackerReportComparators are used in the case that multiple tracks are active (Multiple Tracks
   * == Multiple Goals) Calculate a score for each track (highest score wins) Algorithm is pretty
   * simple.. reward stability, recency
   */
  public static class TrackComparator implements Comparator<TrackReport> {
    // Reward Track for being more stable
    // stability = more frames
    double mStabilityWeight;

    // Lower the age the better the track
    double mCurrentTimestamp;
    double mAgeWeight;

    // Reward Tracks for being continuation of tracks already tracking
    double mSwitchingWeight;
    int mLastTrackID;

    public TrackComparator(
        double curTimestamp,
        double stability_weight,
        double age_weight,
        double switiching_weight,
        int lastTrackID) {
      mStabilityWeight = stability_weight;
      mCurrentTimestamp = curTimestamp;
      mAgeWeight = age_weight;
      mSwitchingWeight = switiching_weight;
      mCurrentTimestamp = curTimestamp;
    }

    double score(TrackReport report) {
      double stability_score = mStabilityWeight * report.stability;
      double age_score =
          mAgeWeight
              * Math.max(
                  0,
                  (Constants.kMaxGoalTrackAge - (mCurrentTimestamp - report.latest_timestamp))
                      / Constants.kMaxGoalTrackAge);
      double switching_score = (report.id == mLastTrackID ? mSwitchingWeight : 0);

      // Compile all of these fields together! this is our score!
      return stability_score + age_score + switching_score;
    }

    // Compare function.. this interfaces to java's built in methods
    @Override
    public int compare(TrackReport o1, TrackReport o2) {
      double ScoreDifference = score(o1) - score(o2);
      // Greater than 0 if o1 is better than o2
      if (ScoreDifference < 0) {
        // o2 is better!
        return 1;
      } else if (ScoreDifference > 1) {
        // o1 is better!
        return -1;
      } else {
        // Tie
        return 0;
      }
    }
  }

  // List of Currrent goals being tracked
  private List<GoalTrack> mCurrentTracks = new ArrayList<>();

  int mNextTrackID = 0;

  int TrackerID = 0;

  public GoalTracker(int tid) {
    TrackerID = tid;
  }

  // Clear out all goal tracks!
  public synchronized void reset() {
    mCurrentTracks.clear();
  }

  // Update goals that we are tracking
  // try to update an existing track.. if not new track
  public synchronized void update(double timestamp, List<Pose2d> field_to_goals) {
    Pose2d CurrentTarget;
    for (int i = 0; i < field_to_goals.size(); i++) {
      CurrentTarget = field_to_goals.get(i);

      // Mark if we und up updating a track
      boolean FoundUpdatedTrack = false;

      // Loop through our current tracks
      GoalTrack CurrentTrackTemp;
      for (int j = 0; j < mCurrentTracks.size(); j++) {
        CurrentTrackTemp = mCurrentTracks.get(j);

        // If we haven't found a track to update
        // attempt to update a track
        if (FoundUpdatedTrack == false) {
          // Attempt to update current track

          if (CurrentTrackTemp.tryUpdate(timestamp, CurrentTarget) == true) {

            // Succesfully Update
            FoundUpdatedTrack = true;
          }
        } else {
          // We have already updated a track
          // perform an empty update
          CurrentTrackTemp.emptyUpdate();
        }
      }

      // If we never found a track to update
      if (FoundUpdatedTrack == false) {
        // Lets create a new goal track
        GoalTrack t = GoalTrack.makeNewTrack(mNextTrackID, timestamp, CurrentTarget);
        t.VisionDistance = CurrentTarget.StoredDistance;

        // Add New Track
        mCurrentTracks.add(t);

        // Incriment our track id!
        mNextTrackID++;
      }
    }

    // Remove Tracks if the track isn't alive
    mCurrentTracks.removeIf(track -> !track.IsGoalTrackAlive());
  }

  // Do we current have tracks
  public synchronized boolean hasTracks() {
    return (!mCurrentTracks.isEmpty());
  }

  public synchronized List<TrackReport> getTrackReports() {
    List<TrackReport> trlist = new ArrayList<>();
    GoalTrack gt;
    for (int i = 0; i < mCurrentTracks.size(); i++) {
      gt = mCurrentTracks.get(i);
      trlist.add(new TrackReport(gt));
    }
    return trlist;
  }
}
