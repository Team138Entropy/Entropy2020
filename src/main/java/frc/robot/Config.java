package frc.robot;
/*
 * Constant values used throughout robot code.
 * In "C" this would be a header file, but alas, this is Java
 */
// @Deprecated
// public class Constants {

import edu.wpi.first.wpilibj.DriverStation;

// System Constants
	// public static double commandLoopIterationSeconds = 0.020;

	// public static boolean practiceBot = false;

	// public final static double CloseLoopJoystickDeadband = 0.2;

	// This is our encoder constant for distance (in METERS) per  encoder pulse
	// 6" Wheels, 15:45 chain drive; 256 encoder counts per drive sprocket rotation
	// public final static double MetersPerPulse = Math.PI*6*.0254*15/45/256;
	// public final static double SecondsTo100Milliseconds = 0.1;

	// Stuff that was deleted that I had to paste back in from the 2018 code
	// public final static int zeroDelay= 60; // Approx 40/sec;
	// public final static double highSpeedModeTriggerThreshold = 0.3;

	// TEST ONLY
	// public final static int LeftDriveEncoderPolarity = -1;
	// public final static int RightDriveEncoderPolarity = 1;

	// 2019 Drive Train constants
	// public final static double FullSpeed = 0.75;
	// public final static double ClimbSpeed = 0.75;

	// Dashboard
	// public static final long DASHBOARD_INTERVAL = 50; // In milliseconds
// }

public class Config{
	public enum Key{
		OI__VISION__ENABLED,

		OI__VISION__POT__MIN,
		OI__VISION__POT__MAX,
		OI__VISION__PID__P,
		OI__VISION__PID__I,
		OI__VISION__PID__D,

		ROBOT__HAS_DRIVETRAIN,
		ROBOT__HAS_TURRET,

		ROBOT__POT__LOCATION,
		ROBOT__POT__RANGE,
		ROBOT__POT__OFFSET,

		ROBOT__TURRET__TALON_LOCATION,

		OI__VISION__PID__MAX_SPEED
	}

	private static Config sInstance;
	public ConfigFile cfg;
	private Config(){
		this.cfg = new ConfigFile();
	}

	public void reload(){
		this.cfg.reload();

        // check that each key is there
        for(Key key : Key.values()){
			try{
				Config.getInstance().getString(key);
			}catch(RuntimeException e){
				DriverStation.reportError("Didn't find key " + key.name() + " in the configuration file or default file. Did you forget to add it?", e.getStackTrace());
				throw new Error("Didn't find key " + key.name() + " in the configuration file or default file. Did you forget to add it?");
			}
        }
	}

	public static synchronized Config getInstance(){
		if(sInstance == null){
			sInstance = new Config();
		}
		return sInstance;
	}

	public String getString(Key key){
		return cfg.getString(key.name());
	}

	public float getFloat(Key key){
		return cfg.getFloat(key.name());
	}

	public double getDouble(Key key){
		return cfg.getDouble(key.name());
	}

	public int getInt(Key key){
		return cfg.getInt(key.name());
	}

	public boolean getBoolean(Key key){
		return cfg.getBoolean(key.name());
	}
} 