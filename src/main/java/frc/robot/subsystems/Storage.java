/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.events.BallDetected;
import frc.robot.events.BallStored;
import frc.robot.events.EventWatcherThread;

/** Add your docs here. */
public class Storage extends Subsystem {

  private static final int ROLLER_PORT = 1;
  private static final int INTAKE_SENSOR_PORT = 1;
  private static final int STORAGE_FULL_SENSOR_PORT = 2;

  // TODO: Tune these values
  private static final double STORE_SPEED = 1;
  private static final double EJECT_SPEED = 1;
  public static final int INTAKE_SENSOR_BALL_THRESHOLD = 375;
  public static final int INTAKE_SENSOR_NO_BALL_THRESHOLD = 100;
  public static final int FULL_SENSOR_BALL_THRESHOLD = 375;
  public static final int FULL_SENSOR_NO_BALL_THRESHOLD = 100;
  public static final int BALLS_EJECTED_DEBOUNCE_THRESHOLD =
      25; // Half a second based on 20 ms per loop

  private WPI_TalonSRX mRoller;
  private AnalogInput mIntakeSensor;
  private AnalogInput mStorageFullSensor;

  private boolean mStoringBall = false;
  private boolean mEjectingBall = false;
  private int mBallsEjectedDebounce = 0;

  // TODO: Hook this up with Autonomous starting with balls
  private int ballsStored = 0;

  private static Storage sInstance;

  public static synchronized Storage getInstance() {
    if (sInstance == null) {
      sInstance = new Storage();
    }
    return sInstance;
  }

  private Storage() {
    mRoller = new WPI_TalonSRX(ROLLER_PORT);
    mIntakeSensor = new AnalogInput(INTAKE_SENSOR_PORT);
    mStorageFullSensor = new AnalogInput(STORAGE_FULL_SENSOR_PORT);
    EventWatcherThread.getInstance().addEvent(new BallDetected());
    EventWatcherThread.getInstance().addEvent((new BallStored()));
  }

  public void periodic() {
    if (mEjectingBall) {
      if (isNoLongerFull()) {
        mBallsEjectedDebounce++;

        // Stop ejecting after the full sensor reports no balls for the debounce threshold
        // period
        if (mBallsEjectedDebounce > BALLS_EJECTED_DEBOUNCE_THRESHOLD) {
          stop();
        }
      } else {
        mBallsEjectedDebounce = 0;
      }
    }
  }

  public boolean isBallDetected() {
    return mIntakeSensor.getValue() > INTAKE_SENSOR_BALL_THRESHOLD;
  }

  public boolean isBallStored() {
    return mStoringBall && mIntakeSensor.getValue() < INTAKE_SENSOR_NO_BALL_THRESHOLD;
  }

  public boolean isFull() {
    return mStorageFullSensor.getValue() > FULL_SENSOR_BALL_THRESHOLD;
  }

  public boolean isNoLongerFull() {
    return mEjectingBall && mStorageFullSensor.getValue() < FULL_SENSOR_NO_BALL_THRESHOLD;
  }

  public void storeBall() {
    mStoringBall = true;
    mRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    mStoringBall = false;
    mEjectingBall = false;
    mRoller.set(ControlMode.PercentOutput, 0);
  }

  public void ejectBalls() {
    mEjectingBall = true;
    mBallsEjectedDebounce = 0;
    mRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  public void increaseBallCount() {
    ballsStored++;
  }

  public void decreaseBallCount() {
    ballsStored--;
  }

  public int getBallCount() {
    return ballsStored;
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}
}
