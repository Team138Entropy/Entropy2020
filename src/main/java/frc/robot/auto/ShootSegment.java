package frc.robot.auto;

import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.Storage;

public class ShootSegment extends Segment {
  private OperatorInterface operatorInterface = OperatorInterface.getInstance();

  private static boolean startShooting = false;
  private static boolean lastStartShooting = false;

  private static boolean stopShooting = false;
  private static boolean lastStopShooting = false;

  // This codebase:
  //                 ##### | #####
  //              # _ _ #|# _ _ #
  //              #      |      #
  //        |       ############
  //                    # #
  // |                  # #
  //                   #   #
  //        |     |    #   #      |        |
  // |  |             #     #               |
  //        | |   |   # .-. #         |
  //                  #( O )#    |    |     |
  // |  ################. .###############  |
  //  ##  _ _|____|     ###     |_ __| _  ##
  // #  |                                |  #
  // #  |    |    |    |   |    |    |   |  #
  //  ######################################
  //                  #     #
  //                   #####
  //              OOOOOOO|OOOOOOO

  public static boolean shouldStartShooting() {
    boolean ret = startShooting && !lastStartShooting;
    lastStartShooting = startShooting;
    return ret;
  }

  public static boolean shouldStopShooting() {
    boolean ret = stopShooting && !lastStopShooting;
    lastStopShooting = stopShooting;
    return ret;
  }

  public static synchronized void resetState() {
    startShooting = false;
    lastStartShooting = false;
    stopShooting = false;
    lastStopShooting = false;
  }

  static synchronized void startShooting() {
    startShooting = true;
  }

  static synchronized void stopShooting() {
    stopShooting = true;
  }

  private boolean done = false;

  @Override
  public void init() {
    logger.info("Initializing shoot segment");

    startShooting();
  }

  @Override
  public void tick() {
    if (Storage.getInstance().isEmpty()) {
      stopShooting();
      done = true;
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Shoot segment finished");
      resetState(); // I can feel the tumors growing
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new ShootSegment();
  }
}
