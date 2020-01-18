package frc.robot.subsystems;
import frc.robot.util.*;
import frc.robot.Kinematics;
import frc.robot.Constants;
import frc.robot.util.geometry.*;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drive extends Subsystem {
    private static Drive mInstance;

    //Drive Talons
    private WPI_TalonSRX  mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    
    //Gear Shifting Solenoid
    //private final Solenoid mShifter;

    //Drive is plummed to default to high gear
    private boolean mHighGear = true;


    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }


    private DriveControlState mDriveControlState;

    private PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }

    


    public synchronized static Drive getInstance(){
        if(mInstance == null){
            mInstance = new Drive();
        }
        return mInstance;
    }


    private Drive(){


        //Shifter Solenoid
        //mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);
       
        mLeftMaster = new WPI_TalonSRX(Constants.kLeftDriveMasterId);
       // configureSpark(mLeftMaster, true, true);

        mLeftSlave = new WPI_TalonSRX(Constants.kLeftDriveSlaveId);
        //configureSpark(mLeftSlave, true, false);

        mRightMaster = new WPI_TalonSRX(Constants.kRightDriveMasterId);
        //configureSpark(mRightMaster, false, true);

        mRightSlave = new WPI_TalonSRX(Constants.kRightDriveSlaveId);
        //configureSpark(mRightSlave, false, false);

        mLeftMaster.configNominalOutputForward(0.,0);
		mLeftMaster.configNominalOutputReverse(0.,0);
		mLeftMaster.configPeakOutputForward(1,0);
		mLeftMaster.configPeakOutputReverse(-1,0);
		mLeftMaster.setNeutralMode(NeutralMode.Brake);
		mLeftMaster.setNeutralMode(NeutralMode.Brake);


        mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);
		mLeftMaster.setSensorPhase(true);
		mLeftMaster.configNominalOutputForward(0.,0);
		mLeftMaster.configNominalOutputReverse(-0.,0);
		mLeftMaster.configPeakOutputForward(1,0);
		mLeftMaster.configPeakOutputReverse(-1,0);
		mLeftMaster.setNeutralMode(NeutralMode.Brake);
		mLeftSlave.setNeutralMode(NeutralMode.Brake);


        // Configure Talon gains
        /*
		mLeftMaster.config_kF(0, Drive_Kf,0);
		mLeftMaster.config_kP(0, Drive_Kp,0);
		mLeftMaster.config_kI(0, Drive_Ki,0);
		mLeftMaster.config_kD(0, Drive_Kd,0);
		mRightMaster.config_kF(0, Drive_Kf,0);
		mRightMaster.config_kP(0, Drive_Kp,0);
		mRightMaster.config_kI(0, Drive_Ki,0);
		mRightMaster.config_kD(0, Drive_Kd,0);
        */

		// Configure slave Talons to follow masters
		mLeftSlave.follow(mLeftMaster);
		mRightSlave.follow(mRightMaster);
        

        


        setOpenLoop(DriveSignal.NEUTRAL);

    }

    public void ZeroSensors(){
        
    }


    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            //setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

    

        signal.PrintLog();
        mLeftMaster.set(ControlMode.PercentOutput, signal.getLeft());
		mRightMaster.set(ControlMode.PercentOutput, signal.getRight() * -1);

    }


    public synchronized void setDrive(double throttle, double wheel, boolean quickTurn) {
        wheel = wheel * -1; //invert wheel
        if(throttle >= .17){
            throttle = .17;
        }

        if(throttle <= -.17){
            throttle = -.17;
        }
        
        

        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
            quickTurn = true;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    /*
        SwitchGears
        Before gears are switched, it would be a good idea to check
        psi to ensure we can properly drive the piston
    */
    public synchronized void SwitchGears(boolean HighGear){

    }


    /*
        Test all Sensors in the Subsystem
    */
    public void CheckSubsystems(){

    }


    public synchronized double getLeftEncoderDistance(){
        return 0.0;
    }

    public synchronized double getRightEncoderDistance(){
        return 0.0;
    }

    public synchronized Rotation2d getRotation(){
        return null;
    }

    public double getLeftLinearVelocity() {
        return 0;
    }
    
    public double getRightLinearVelocity(){
        return 0;
    }

  



}