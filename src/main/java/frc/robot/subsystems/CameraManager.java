/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* This is the camera manager code from last years (2019) robot.              */
/* It has some adaptations and changes since the 2019 code was mashed together*/
/* in one file.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Logger;

// camera subsystem
public class CameraManager extends Subsystem {
  private static UsbCamera frontCamera;
  private static UsbCamera backCamera;
  private static CameraManager sInstance;

  Logger mLogger = new Logger("cameraManager");

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
    frontCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

    // back camera
    backCamera = CameraServer.getInstance().startAutomaticCapture("backCamera", 1);
    // another resolution set
    backCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystems() {
    if (frontCamera.isEnabled()) {
      mLogger.info("Front camera ready");
    } else {
      mLogger.warn(
          "Front camera not enabled " + frontCamera.isConnected() + " " + frontCamera.isValid());
    }

    if (backCamera.isEnabled()) {
      mLogger.info("Back camera ready");
    } else {
      mLogger.warn(
          "Back camera not enabled " + backCamera.isConnected() + " " + backCamera.isValid());
    }
  }
}
