package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap; 
import java.util.Map; 
  

/*
   Manages all vision tracking cameras
   UDP Packets will be streamed back from tracking camera(s) for speed
   Runs in its own thread

*/
public class VisionManager extends Subsystem {
    private static VisionManager mInstance;

    //private Set<int, RPICamera> mCameras;
    
   

    //Add Multiple Cameras
    enum ActiveCamera {
        Main
    }

    //Default to the Main Cammera
    private ActiveCamera mActiveCameraValue = ActiveCamera.Main;
    private RPICamera mActiveCamera;

     private VisionManager(){


        //Intialize set of Cameras
        //mCameras = new HashMap<>();
        //mCameras.put(ActiveCamera.Main, new RPICamera());

        //Default Active Cammera
        //UpdateActiveCamera(mActiveCameraValue);
    }


    public synchronized static VisionManager getInstance(){
        if(mInstance == null){
            mInstance = new VisionManager();
        }
        return mInstance;
    }

    public void ZeroSensors(){
        
    }


    public void UpdateActiveCamera(int value){
        //mActiveCamera = mCameras.get(value);
    }



    /*
        Test all Sensors in the Subsystem
    */
    public void CheckSubsystems(){

    }





}