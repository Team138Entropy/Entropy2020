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
  private static UsbCamera mCamera;
  private static CameraManager sInstance;
  private boolean mIsClimbingCamera = false;

  Logger mLogger = new Logger("cameraManager");

  // this is a singleton
  public static synchronized CameraManager getInstance() {
    if (sInstance == null) {
      sInstance = new CameraManager();
    }
    return sInstance;
  }

  private CameraManager() {}

  private void setupCamera(int id){
    try {
      // front camera
      mCamera = CameraServer.getInstance().startAutomaticCapture("frontCamera", id);
      // resolution set
      mCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    } catch (Exception exception) {
      mLogger.error("Camera Initialization failed");
    }
  }

  public void init() {
    setupCamera(0);
  }

  public void setWhichCamera(boolean wantsClimbingCamera){
    mCamera.close();
    if(wantsClimbingCamera && !mIsClimbingCamera){
      setupCamera(1);
    }else if(mIsClimbingCamera){
      setupCamera(0);
    }

    // update whether we currently have a climbing camera
    mIsClimbingCamera = wantsClimbingCamera;
  }

  public boolean getCameraStatus(){
    return mCamera.isConnected() && mCamera.isEnabled() && mCamera.isValid();
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void checkSubsystem() {
    if (mCamera.isEnabled()) {
      mLogger.info("Front camera ready");
    } else {
      mLogger.warn(
          "Front camera not enabled " + mCamera.isConnected() + " " + mCamera.isValid());
    }
  }
}
