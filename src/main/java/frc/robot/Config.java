package frc.robot;
/*
 * Constant values used throughout robot code.
 * In "C" this would be a header file, but alas, this is Java
 */
// @Deprecated
// public class Constants {

// TODO: Address performance concerns by profiling execution time and trimming calls
public class Config {

  public enum Key {
    DRIVE__PEAK_OUTPUT_CLIMBING(0.3),

    // Physical qualities of robot
    ROBOT__REAL_TRACK_WIDTH(0.58d), // Meters, 0.58 is about 23"
    DRIVE__TICKS_PER_METER(640),

    // Motors,
    DRIVE__LEFT_BACK_PORT(1),
    DRIVE__LEFT_FRONT_PORT(2),
    DRIVE__RIGHT_BACK_PORT(3),
    DRIVE__RIGHT_FRONT_PORT(4),
    INTAKE__ROLLER(10),
    STORAGE__BOTTOM_ROLLER(8),
    STORAGE__TOP_ROLLER(7),
    SHOOTER__ROLLER(6),
    SHOOTER__ROLLER_SLAVE(5),
    CLIMBER__MOTOR(11),

    // Sensors
    INTAKE__SENSOR(1),

    // Speeds
    INTAKE__ROLLER_SPEED(1d),
    STORAGE__ROLLER_STORE_SPEED(1d),
    STORAGE__ROLLER_SPEED_FACTOR(0.5d),
    STORAGE__ROLLER_BOTTOM_SPEED_FACTOR(1.25d),
    STORAGE__ROLLER_EJECT_SPEED(1d),
    CLIMBER__EXTEND_SPEED(1d),
    CLIMBER__RETRACT_SPEED(-1d),
    CLIMBER__HOME_SPEED(.5d),
    DRIVE__FORWARD_ACCEL_RAMP_TIME_SECONDS(1d),
    DRIVE__REVERSE_BRAKE_RAMP_TIME_SECONDS(1d),
    CLIMBER__JOG_SPEED_FACTOR(1.0d),

    CLIMBER__ENABLED(true),

    // Auto config (temporary)
    AUTO__SELECTED_PATH("test"),

    // PID Stuff
    OI__VISION__POT__MIN(0),
    OI__VISION__POT__MAX(100d),
    OI__VISION__PID__P(0.3d),
    OI__VISION__PID__I(0.003d),
    OI__VISION__PID__D(0),

    AUTO__DRIVE_PID_P(1),
    AUTO__DRIVE_PID_I(0),
    AUTO__DRIVE_PID_D(0),
    AUTO__DRIVE_PID_ACCEPTABLE_ERROR(0.5),
    AUTO__DRIVE_PID_RAMP(0.2),

    AUTO__TURN_PID_P(0.4d),
    AUTO__TURN_PID_I(0d),
    AUTO__TURN_PID_D(0d),
    AUTO__TURN_PID_ACCEPTABLE_ERROR(5d),
    AUTO__TURN_PID_MAX(0.5),

    ROBOT__HAS_DRIVETRAIN(true),
    ROBOT__HAS_TURRET(false),
    ROBOT__HAS_LEDS(false),

    ROBOT__PRACTICE_JUMPER_PIN(1),

    ROBOT__POT__LOCATION(0),
    ROBOT__POT__RANGE(-6),
    ROBOT__POT__OFFSET(321.8d),

    ROBOT__TURRET__TALON_LOCATION(1),

    CLIMBER__OVERCURRENT_THRESHOLD(12d),
    CLIMBER__OVERCURRENT_MIN_OCCURENCES(25),
    CLIMBER__OVERCURRENT_COUNTDOWN_LENGTH(30d),
    SHOOTER__VELOCITY_ADJUSTMENT(100),

    CLIMBER__DETECT_BAND(100),
    CLIMBER__EXTENDED_HEIGHT_IN_ENCODER_TICKS(70000),
    CLIMBER__RETRACTED_HEIGHT_IN_ENCODER_TICKS(40000),
    STORAGE__BALL_DISTANCE_IN_ENCODER_TICKS_PRACTICE(2000d),
    STORAGE__BALL_DISTANCE_IN_ENCODER_TICKS_PRODUCTION(2000d),

    OI__VISION__PID__MAX_SPEED(0.25d),

    CLIMBER__PID_LOOP_INDEX(0),
    CLIMBER__TIMEOUT_MS(10),
    CLIMBER__KF(0),
    CLIMBER__KP(.3d),
    CLIMBER__KI(0d),
    CLIMBER__KD(0d),

    TESTMODE__TIME_PER_TEST(1.0d),
    TESTMODE__EXPECTED_STORAGE_DISTANCE(550),
    TESTMODE__STORAGE_ACCEPTABLE_ERROR(50),
    TESTMODE__EXPECTED_SHOOTER_SPEED(1110),
    TESTMODE__SHOOTER_ACCEPTABLE_ERROR(100);

    private Object value;

    private Key(Double k) {
      value = k;
    }

    private Key(Float k) {
      value = k;
    }

    private Key(Boolean k) {
      value = k;
    }

    private Key(Integer k) {
      value = k;
    }

    private Key(String k) {
      value = k;
    }

    public Object getValue() {
      return value;
    }
  }

  Logger mLogger = new Logger("Config");

  private static Config sInstance;
  public ConfigFile cfg;

  private Config() {
    this.cfg = new ConfigFile();
  }

  public void reload() {
    this.cfg.reload();

    // check that each key is there
    for (Key key : Key.values()) {
      Object valueFromConfig = null;

      try {
        valueFromConfig = cfg.getString(key.name());
      } catch (Exception exception) {
        mLogger.error("Malformed configuration");
      }

      if (valueFromConfig == null) {
        mLogger.warn(
            "Didn't find key " + key.name() + " in the configuration file. Using a default.");
      }
    }
  }

  public static synchronized Config getInstance() {
    if (sInstance == null) {
      sInstance = new Config();
    }
    return sInstance;
  }

  public String getString(Key key) {
    String value = null;
    try {
      value = cfg.getString(key.name());
    } catch (Exception exception) {
      mLogger.warn("Key " + key.name() + " was not found in the config file! Using default value.");
      value = (String) key.value;
    }
    return value;
  }

  public float getFloat(Key key) {
    Float value = null;
    try {
      value = cfg.getFloat(key.name());
    } catch (Exception exception) {
      mLogger.warn("Key " + key.name() + " was not found in the config file! Using default value.");
      value = (Float) key.value;
    }
    return value;
  }

  public double getDouble(Key key) {
    Double value = null;
    try {
      value = cfg.getDouble(key.name());
    } catch (Exception exception) {
      mLogger.warn("Key " + key.name() + " was not found in the config file! Using default value.");
      value = (Double) key.value;
    }
    return value;
  }

  public int getInt(Key key) {
    Integer value = null;
    try {
      value = cfg.getInt(key.name());
    } catch (Exception exception) {
      mLogger.warn("Key " + key.name() + " was not found in the config file! Using default value.");
      value = (Integer) key.value;
    }
    return value;
  }

  public boolean getBoolean(Key key) {
    Boolean value = null;
    try {
      value = cfg.getBoolean(key.name());
    } catch (Exception exception) {
      mLogger.warn("Key " + key.name() + " was not found in the config file! Using default value.");
      value = (Boolean) key.value;
    }
    return value;
  }
}
