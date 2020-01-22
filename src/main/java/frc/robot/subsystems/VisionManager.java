package frc.robot.subsystems;

/*
   Manages all vision tracking cameras
   UDP Packets will be streamed back from tracking camera(s) for speed
   Runs in its own thread

*/
public class VisionManager extends Subsystem {
  private static VisionManager mInstance;

  public static synchronized VisionManager getInstance() {
    if (mInstance == null) {
      mInstance = new VisionManager();
    }
    return mInstance;
  }

  // Add Multiple Cameras
  enum ActiveCamera {
    Main
  }

  // Default to the Main Cammera
  private ActiveCamera mActiveCameraValue = ActiveCamera.Main;
  private RPICamera mActiveCamera;

  private RPICamera mShooterCamera;
  private RPICamera mIntakeCamera;

  private VisionManager() {
    // Start a Socket to listen to UDP Packet
    // Each thread pass to a processer which

  }

  /*
      Process every single packet
  */
  private void ProcessPacket() {
    try {

    } catch (Exception e) {

    }
  }

  public void ZeroSensors() {}

  public void UpdateActiveCamera(int value) {
    // mActiveCamera = mCameras.get(value);
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void CheckSubsystems() {}
}
