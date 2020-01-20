package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RPICamera {

    private NetworkTable mNetworkTable;

    public RPICamera(String CameraName) {
        mNetworkTable = NetworkTableInstance.getDefault().getTable(CameraName);

        // Spawn Thread to periodically update values

    }

    // Run in a seperate thread that peridically
    public void QueryValues() {}
}
