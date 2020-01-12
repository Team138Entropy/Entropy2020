package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Logger class lets you log messages and only have them show up if a config
 * file entry enables it. It supports the following logging levels: - ERROR -
 * WARN - INFO (the default) - VERBOSE - DEBUG - SILLY You can select the minimum level at
 * which to allow logging. For example, if log.myLogger is INFO, then only logs
 * with the levels of WARN, ERROR, or INFO will show up. Set the log level in
 * the config to OFF (or don't set it at all) to disable logging for this logger
 * entirely. See Config.java for more info about configuration.
 * 
 * @example Logger myLogger = new Logger("myLogger");
 * 
 *          // if log.myLogger is INFO or lower, this prints to STDOUT
 *          [myLogger:INFO] Hello from myLogger! myLogger.log("Hello from
 *          myLogger!");
 * 
 *          // prints to STDERR [myLogger:ERR] Error from myLogger!
 *          myLogger.err("Error from myLogger!");
 */
public class Logger {
    // our supported logging levels, most important first
    public enum SupportedLevels{
        ERROR,
        WARN,
        INFO,
        VERBOSE,
        DEBUG,
        SILLY
    };

    String logPath;
    /**
     * Initiates the logger with the path of logPath.
     * @param logPath
     */
    public Logger(String logPath){
        this.logPath = logPath;
    }


    /**
     * This is an internal method that determines how log file entries are formatted. You can @Override this method to change the output.
     * @param logPath 
     * @param thisLogLevel
     * @param message
     * @return
     */
    public static String getLogMsg(String logPath, SupportedLevels thisLogLevel, String message){
        return "[" + logPath + ":" + thisLogLevel.name() + "] " + message;
    }

    /**
     * Logs to a given level
     * @param level The level to log to
     * @param message The message to log
     */
    public void logLevel(SupportedLevels level, String message){
        String minLevel = Config.getInstance().cfg.getString("LOG__" + logPath.toUpperCase());

        // if logging is enabled at all for this logger,
        // and the level is recognized,
        // and our level is higher-up or equal to the minimum specified,
        // then log it
        if(minLevel != null && minLevel != "OFF" && Arrays.asList(SupportedLevels.values()).indexOf(level) <= Arrays.asList(SupportedLevels.values()).indexOf(SupportedLevels.valueOf(minLevel))){
            // if the level we're logging at is WARN or ERR, then log to STDERR, otherwise log to STDOUT
            if(level == SupportedLevels.ERROR){
                DriverStation.reportError(getLogMsg(logPath, level, message), Thread.currentThread().getStackTrace());
            }else if(level == SupportedLevels.WARN){
                DriverStation.reportWarning(getLogMsg(logPath, level, message), Thread.currentThread().getStackTrace());
            }else{
                System.out.println(getLogMsg(logPath, level, message));
            }
        }
    }

    /**
     * Logs at the level ERROR
     * @param message The message to log
     */
    public void err(String message){
        logLevel(SupportedLevels.ERROR, message);
    }
    
    /**
     * Logs at the level ERROR
     * @param message The message to log
     */
    public void error(String message){
        logLevel(SupportedLevels.ERROR, message);
    }
    
    /**
     * Logs at the level WARN
     * @param message The message to log
     */
    public void warn(String message){
        logLevel(SupportedLevels.WARN, message);
    }

    /**
     * Logs at the default level, INFO
     * @param message The message to log
     */
    public void log(String message){
        logLevel(SupportedLevels.INFO, message);
    }

    /**
     * Logs at the level INFO
     * @param message The message to log
     */
    public void info(String message){
        logLevel(SupportedLevels.INFO, message);
    }

    /**
     * Logs at the level DEBUG
     * @param message The message to log
     */
    public void debug(String message){
        logLevel(SupportedLevels.DEBUG, message);
    }

    /**
     * Logs at the level VERBOSE
     * @param message The message to log
     */
    public void verbose(String message){
        logLevel(SupportedLevels.VERBOSE, message);
    }

    /**
     * Logs at the level SILLY
     * @param message The message to log
     */
    public void silly(String message){
        logLevel(SupportedLevels.SILLY, message);
    }
}
