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
    OI__VISION__ENABLED(false),

    OI__VISION__POT__MIN(0),
    OI__VISION__POT__MAX(100d),
    OI__VISION__PID__P(0.3d),
    OI__VISION__PID__I(0.003d),
    OI__VISION__PID__D(0),

    ROBOT__HAS_DRIVETRAIN(true),
    ROBOT__HAS_TURRET(false),
    ROBOT__HAS_LEDS(false),

    ROBOT__POT__LOCATION(0),
    ROBOT__POT__RANGE(-6),
    ROBOT__POT__OFFSET(321.8d),

    ROBOT__TURRET__TALON_LOCATION(1),

    OI__VISION__PID__MAX_SPEED(0.25d);

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

  private static Config sInstance;
  public ConfigFile cfg;

  private Config() {
    this.cfg = new ConfigFile();
  }

  public void reload() {
    this.cfg.reload();

    // check that each key is there
    for (Key key : Key.values()) {
      Object valueFromConfig = cfg.getString(key.name());

      if (valueFromConfig == null) {
        DriverStation.reportError(
            "Didn't find key "
                + key.name()
                + " in the configuration file. Did you forget to add it? Using a default anyway...",
            new RuntimeException().getStackTrace());
      }
    }
  }

  public static synchronized Config getInstance() {
    if (sInstance == null) {
      sInstance = new Config();
    }
    return sInstance;
  }

  /*String value = null;
  try{
    value = cfg.getString(key.name());
  }catch(NumberFormatException ignored){}

  if(key.value instanceof String){// the key we're trying to get has the same type as what the key is defined to be
    if(value == null){// the value wasn't found in the file, so let's return a default
      DriverStation.reportError("Key " + key.name() + " was not found in the config file! You might have mistyped something, please fix this! A default value was used.", new RuntimeException().getStackTrace());
      return (String) key.value;
    }else{

      return value;
    }
  }else{
    DriverStation.reportError("Key " + key.name() + " not the correct type. Got " + key.value, new RuntimeException().getStackTrace());
    throw new RuntimeException("Read the driver station");
  } */

  public String getString(Key key) {
    String value = null;
    try {
      value = cfg.getString(key.name());
    } catch (NumberFormatException ignored) {
    }

    if (value != null) {
      // everything's good
      return value;
    } else { // the value wasn't found in the file, so let's return a default
      DriverStation.reportError(
          "Key "
              + key.name()
              + " was not found in the config file! You might have mistyped something, please fix this! A default value was used.",
          new RuntimeException().getStackTrace());
      return (String) key.value;
    }
  }

  public float getFloat(Key key) {
    Float value = null;
    try {
      value = cfg.getFloat(key.name());
    } catch (NumberFormatException ignored) {
    }

    if (value != null) {
      // everything's good
      return value;
    } else { // the value wasn't found in the file, so let's return a default
      DriverStation.reportError(
          "Key "
              + key.name()
              + " was not found in the config file! You might have mistyped something, please fix this! A default value was used.",
          new RuntimeException().getStackTrace());
      return (Float) key.value;
    }
  }

  public double getDouble(Key key) {
    Double value = null;
    try {
      value = cfg.getDouble(key.name());
    } catch (NumberFormatException ignored) {
    }

    if (value != null) {
      // everything's good
      return value;
    } else { // the value wasn't found in the file, so let's return a default
      DriverStation.reportError(
          "Key "
              + key.name()
              + " was not found in the config file! You might have mistyped something, please fix this! A default value was used.",
          new RuntimeException().getStackTrace());
      return (Double) key.value;
    }
  }

  public int getInt(Key key) {
    Integer value = null;
    try {
      value = cfg.getInt(key.name());
    } catch (NumberFormatException ignored) {
    }

    if (value != null) {
      // everything's good
      return value;
    } else { // the value wasn't found in the file, so let's return a default
      DriverStation.reportError(
          "Key "
              + key.name()
              + " was not found in the config file! You might have mistyped something, please fix this! A default value was used.",
          new RuntimeException().getStackTrace());
      return (Integer) key.value;
    }
  }

  public boolean getBoolean(Key key) {
    Boolean value = null;
    try {
      value = cfg.getBoolean(key.name());
    } catch (NumberFormatException ignored) {
    }

    if (value != null) {
      // everything's good
      return value;
    } else { // the value wasn't found in the file, so let's return a default
      DriverStation.reportError(
          "Key "
              + key.name()
              + " was not found in the config file! You might have mistyped something, please fix this! A default value was used.",
          new RuntimeException().getStackTrace());
      return (Boolean) key.value;
    }
  }
}
