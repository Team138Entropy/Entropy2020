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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.events.BallDetected;
import frc.robot.events.BallStored;
import frc.robot.events.EventWatcherThread;

/** Add your docs here. */
public class Storage extends Subsystem {

  private static final int ROLLER_PORT = 1;
  private static final int INTAKE_SENSOR_PORT = 1;

  private static final int STORAGE_CAPICTY = 5;

  // TODO: Tune these values
  private static final double STORE_SPEED = 1;
  private static final double EJECT_SPEED = 1;
  public static final int INTAKE_SENSOR_BALL_THRESHOLD = 375;
  public static final int INTAKE_SENSOR_NO_BALL_THRESHOLD = 100;
  public static final int EJECT_DELAY_SECONDS = 5;

  private WPI_TalonSRX mRoller;
  private AnalogInput mIntakeSensor;
  private Timer mEjectTimer;

  private int mBallCount = 0;

  // State variables
  public enum State {
    IDLE,
    STORING,
    EJECTING
  }

  private State mState = State.IDLE;

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
    EventWatcherThread.getInstance().addEvent(new BallDetected());
    EventWatcherThread.getInstance().addEvent((new BallStored()));
    mEjectTimer = new Timer();
  }

  public void periodic() {
    // Check if we're done ejecting
    if (mState == State.EJECTING && mEjectTimer.get() >= EJECT_DELAY_SECONDS) {
      mEjectTimer.stop();
      mEjectTimer.reset();
      stop();
    }
  }

  public boolean isBallDetected() {
    return mIntakeSensor.getValue() >= INTAKE_SENSOR_BALL_THRESHOLD;
  }

  public boolean isBallStored() {
    return (mState == State.STORING)
        && (mIntakeSensor.getValue() <= INTAKE_SENSOR_NO_BALL_THRESHOLD);
  }

  public boolean isEmpty() {
    return mBallCount == 0;
  }

  public boolean isFull() {
    return mBallCount == STORAGE_CAPICTY;
  }

  public void storeBall() {
    mState = State.STORING;
    mRoller.set(ControlMode.PercentOutput, STORE_SPEED);
  }

  /** Stops the roller. */
  public void stop() {
    mState = State.IDLE;
    mRoller.set(ControlMode.PercentOutput, 0);
  }

  public void ejectBalls() {
    mState = State.EJECTING;
    mEjectTimer.reset();
    mEjectTimer.start();
    mRoller.set(ControlMode.PercentOutput, EJECT_SPEED);
  }

  @Override
  public void ZeroSensors() {}

  @Override
  public void CheckSubsystems() {}
}
