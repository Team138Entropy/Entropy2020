package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;


public class CameraManager extends Subsystem {
  private static UsbCamera frontCamera;
  private static UsbCamera backCamera;
  private static CameraManager sInstance;

  public static synchronized CameraManager getInstance() {
    if (sInstance == null) {
      sInstance = new CameraManager();
    }
    return sInstance;
  }

  private CameraManager() {
    frontCamera = CameraServer.getInstance().startAutomaticCapture("frontCamera", 0);
    frontCamera.setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);
    backCamera = CameraServer.getInstance().startAutomaticCapture("backCamera", 1);
    backCamera.setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);
  }
  public void CheckSubsystems(){}
  public void ZeroSensors(){}
}