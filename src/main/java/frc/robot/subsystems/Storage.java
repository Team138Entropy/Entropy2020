/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.events.BallDetected;
import frc.robot.events.BallStored;
import frc.robot.events.EventWatcherThread;

/**
 * Add your docs here.
 */
public class Storage extends Subsystem{

    private static final int ROLLER_PORT = 1;

    // TODO: Tune these values
    private static final double ROLLER_SPEED = 1;
   
    private WPI_TalonSRX mRoller;

    private static Storage sInstance;
    
    public static synchronized Storage getInstance() {
        if (sInstance == null) {
            sInstance = new Storage();
        }
        return sInstance;
    }

    private Storage() {
        mRoller = new WPI_TalonSRX(ROLLER_PORT);

        EventWatcherThread.getInstance().addEvent(new BallDetected());
        EventWatcherThread.getInstance().addEvent((new BallStored()));
    }

    public boolean isBallDetected(){
        //TODO implement
        return false;
    }

    public boolean isBallStored() {
        //TODO implement
        return false;
    }
    
    public void start() {
        mRoller.set(ControlMode.PercentOutput, ROLLER_SPEED);
    }

    /**
     * Stops the roller.
     */
    public void stop() {
        mRoller.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void ZeroSensors() {}

    @Override
    public void CheckSubsystems() {}
}
