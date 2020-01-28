/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

// camera subsystem
public class CameraManager extends Subsystem {
  private static UsbCamera frontCamera;
  private static UsbCamera backCamera;
  private static CameraManager sInstance;

  // this is a singleton
  public static synchronized CameraManager getInstance() {
    if (sInstance == null) {
      sInstance = new CameraManager();
    }
    return sInstance;
  }


  private CameraManager() {
    // front camera 
    frontCamera = CameraServer.getInstance().startAutomaticCapture("frontCamera", 0);
    // resolution set
    frontCamera.setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);

    // back camera
    backCamera = CameraServer.getInstance().startAutomaticCapture("backCamera", 1);
    // another resolution set
    backCamera.setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);
  }

  public void CheckSubsystems(){}
  public void ZeroSensors(){}
}
