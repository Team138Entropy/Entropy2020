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

    public synchronized static VisionManager getInstance(){
        if(mInstance == null){
            mInstance = new VisionManager();
        }
        return mInstance;
    }
   

    //Add Multiple Cameras
    enum ActiveCamera {
        Main
    }

    //Default to the Main Cammera
    private ActiveCamera mActiveCameraValue = ActiveCamera.Main;
    private RPICamera mActiveCamera;

     private VisionManager(){
        //Initalize UDP Socket
        //Intercept all TCP Packets

    
    }


    /*
        Process every single packet
    */
    private void ProcessPacket(){
        try{

        }catch(Exception e){

        }
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