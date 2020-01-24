package frc.robot;
/*
 * Constant values used throughout robot code.
 * In "C" this would be a header file, but alas, this is Java
 */
// @Deprecated
// public class Constants {

import edu.wpi.first.wpilibj.DriverStation;

public class Config {
  public enum Key {
    OI__VISION__ENABLED,

    OI__VISION__POT__MIN,
    OI__VISION__POT__MAX,
    OI__VISION__PID__P,
    OI__VISION__PID__I,
    OI__VISION__PID__D,

    ROBOT__HAS_DRIVETRAIN,
    ROBOT__HAS_TURRET,
    ROBOT__HAS_LEDS,

    ROBOT__POT__LOCATION,
    ROBOT__POT__RANGE,
    ROBOT__POT__OFFSET,

    ROBOT__TURRET__TALON_LOCATION,

    OI__VISION__PID__MAX_SPEED
  }

  private static Config sInstance;
  public ConfigFile cfg;

  private Config() {
    this.cfg = new ConfigFile();
  }

  public void reload() {
    this.cfg.reload();

    // check that each key is there
    for (Key key : Key.values()) {
      try {
        Config.getInstance().getString(key);
      } catch (RuntimeException e) {
        DriverStation.reportError(
            "Didn't find key "
                + key.name()
                + " in the configuration file or default file. Did you forget to add it?",
            e.getStackTrace());
        throw new Error(
            "Didn't find key "
                + key.name()
                + " in the configuration file or default file. Did you forget to add it?");
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
    return cfg.getString(key.name());
  }

  public float getFloat(Key key) {
    return cfg.getFloat(key.name());
  }

  public double getDouble(Key key) {
    return cfg.getDouble(key.name());
  }

  public int getInt(Key key) {
    return cfg.getInt(key.name());
  }

  public boolean getBoolean(Key key) {
    return cfg.getBoolean(key.name());
  }
}
