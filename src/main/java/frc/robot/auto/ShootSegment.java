package frc.robot.auto;

import frc.robot.OI.OperatorInterface;
import frc.robot.subsystems.Storage;

public class ShootSegment extends Segment {
  private OperatorInterface operatorInterface = OperatorInterface.getInstance();

  private static boolean startShooting = false;
  private static boolean stopShooting = false;

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
    return startShooting;
  }

  public static boolean shouldStopShooting() {
    return stopShooting;
  }

  public static synchronized void resetState() {
    
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

    //operatorInterface.overrideShoot(); // Simulate first button press
    startShooting();

  }

  @Override
  public void tick() {
    if (Storage.getInstance().isEmpty()) {
      // operatorInterface.overrideShoot(); // Simulate last button press
      stopShooting();
      done = true;
    }
  }

  @Override
  public boolean finished() {
    if (done) {
      logger.info("Shoot segment finished");
    }

    return done;
  }

  @Override
  public Segment copy() {
    return new ShootSegment();
  }
}
