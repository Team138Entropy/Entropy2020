package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.util.drivers.MotorChecker;
import frc.robot.util.drivers.TalonSRXChecker;
import frc.robot.util.drivers.TalonSRXUtil;
import frc.robot.util.LatchedBoolean;
import edu.wpi.first.wpilibj.AnalogInput;

import java.util.ArrayList;

public class Turret2 extends ServoMotorSubsystem {
    private static Turret2 mInstance;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mHoming = false;
    public static final boolean kUseManualHomingRoutine = false;

    public synchronized static Turret2 getInstance() {
        if (mInstance == null) {
            mInstance = new Turret2(Constants.kTurretConstants);
        }

        return mInstance;
    }

    private Turret2(final ServoMotorSubsystemConstants constants) {
        super(constants);
        TalonSRXUtil.checkError(
                mMaster.configClosedLoopPeakOutput(1, 0.8, Constants.kLongCANTimeoutMs),
                "Unable to configure close loop peak output for turret!");
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean atHomingLocation() {
        // Banner seems to sway between 3.2 and 5.0
       // return mBannerInput.getAverageVoltage() > 4.0;
        return false;
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset) && kUseManualHomingRoutine) {
            System.out.println("Turret going into home mode!");
            mHoming = true;
            mMaster.overrideSoftLimitsEnable(false);
        }
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mHoming) {
            if (atHomingLocation()) {
                if (mPeriodicIO.demand > 0) {
                    mMaster.setSelectedSensorPosition((int) unitsToTicks(-2.0));
                } else {
                    mMaster.setSelectedSensorPosition((int) unitsToTicks(0.26));
                }
                mMaster.overrideSoftLimitsEnable(true);
                System.out.println("Homed!!!");
                mHoming = false;
            }

            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                        0.0);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward,
                        0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }

    }

    @Override
    public void checkSubsystem() {
        /*
        return TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                    private static final long serialVersionUID = 1636612675181038895L;

					{
                        add(new MotorChecker.MotorConfig<>("master", mMaster));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mRunOutputPercentage = 0.1;
                        mRunTimeSec = 1.0;
                        mCurrentFloor = 0.1;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = mMaster::getSelectedSensorVelocity;
                    }
                });
                */
    }
}