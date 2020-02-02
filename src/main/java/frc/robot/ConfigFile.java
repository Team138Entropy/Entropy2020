package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * This is a class that lets us configure settings using a java .properties, keeping them in source
 * control but also allows changes to be deployed without having to do a full code build & deploy.
 * Use deployconfig.bat, deployconfig.ps1, or deployconfig.sh to deploy configuration file changes
 * to a connected robot. This class should only be instantiated once.
 */
public class ConfigFile {
  Properties mProps;

  /** Reads the configuration file. */
  public ConfigFile() {
    reload();
  }

  /**
   * Reloads the configuration file. Run it after changing the file's contents. It will run when the
   * constructor is initalized (no need to manually run it the first time).
   */
  public void reload() {
    // load the config
    try (InputStream input =
        new FileInputStream(Filesystem.getDeployDirectory() + "/config.properties")) {
      mProps = new Properties();

      // load the properties file
      mProps.load(input);

    } catch (IOException ex) {
      DriverStation.reportError(ex.getMessage(), false);
    }
  }

  /**
   * @param key The name of the property. It should be camelCase and use periods for depth, eg.
   *     user.name
   * @return The value of the property as a String.
   */
  public String getProp(String key) {
    String value = mProps.getProperty(key);
    return value;
  }

  public String getString(String key) {
    return getProp(key);
  }

  /**
   * @param key The name of the property. It should be camelCase and use periods for depth, eg.
   *     user.name
   * @return The value of the property casted to a float.
   */
  public float getFloat(String key) {
    return Float.parseFloat(getProp(key));
  }

  /**
   * @param key The name of the property. It should be camelCase and use periods for depth, eg.
   *     user.name
   * @return The value of the property casted to an int.
   */
  public int getInt(String key) {
    return Integer.parseInt(getProp(key));
  }

  /**
   * @param key The name of the property. It should be camelCase and use periods for depth, eg.
   *     user.name
   * @return The value of the property casted to a boolean.
   */
  public boolean getBoolean(String key) {
    return Boolean.parseBoolean(getProp(key));
  }

  /**
   * @param key The name of the property. It should be camelCase and use periods for depth, eg.
   *     user.name
   * @return The value of the property casted to a double.
   */
  public double getDouble(String key) {
    return Double.parseDouble(getProp(key));
  }
}
