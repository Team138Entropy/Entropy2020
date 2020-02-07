package frc.robot.subsystems;


public class Turret2 {
    private static Turret2 mInstance;

    public static Turret2 getInstance(){
        if(mInstance == null){
            mInstance = new Turret2();
        }
        return mInstance;
    }

    



}