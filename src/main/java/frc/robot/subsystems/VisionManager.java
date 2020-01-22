package frc.robot.subsystems;

import frc.robot.RobotState;
import frc.robot.vision.TargetInfo;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

// UDP Reciever used in vision manager
class UDPReciever {
    DatagramSocket recieveSocket = null;
    byte[] receiveData = new byte[2048];
    DatagramPacket recievePacket = null;

    /**
     * Constructor for the UDP reciever. Sets up internal memory structures in prep to start
     * listening for packets.
     *
     * @param listen_to_addr String of the IP address of the coprocessor (For example, "10.17.36.8")
     * @param listen_on_port integer port number to listen on. Often between 5800 and 5810 per FMS
     *     whitepaper. Must match whatever port the coprocessor is sending information to.
     */
    public UDPReciever(String listen_from_addr_in, int listen_on_port_in) {
        try {
            recieveSocket = new DatagramSocket(listen_on_port_in);
            recievePacket = new DatagramPacket(receiveData, receiveData.length);
            recieveSocket.setSoTimeout(10);
        } catch (IOException e) {
            System.out.println("Error: Cannot set up UDP reciever socket: " + e.getMessage());
            recieveSocket = null;
        }
    }

    /**
     * Listens on the UPD connection for a packet. Casts it into a java string and returns it. Note
     * this method will block while <i> aggressively polling </i> until a full packet is received,
     * and drop all packets but the most recent.
     *
     * @return String of the data acquired from the UDP connection (if any data gotten)
     */
    public String getPacket() {
        boolean last_packet = false;
        String rx_string = "";
        if (recieveSocket != null) {
            while (last_packet == false) {
                try {
                    recieveSocket.receive(recievePacket);
                    rx_string = new String(recievePacket.getData(), 0, recievePacket.getLength());
                } catch (java.net.SocketTimeoutException e) {
                    /* timeout exception - this is OK, just means we don't see new complete packet. */
                    if (rx_string.length() != 0) {
                        // We have a packet and there are no more in the recieve queue. Break and
                        // return the last packet one.
                        last_packet = true;
                        // System.out.println(rx_string);
                    }
                } catch (IOException e) {
                    /* some other error we didn't think about... don't try to listen anymore */
                    System.out.println("Error: Cannot get data from UDP socket: " + e.getMessage());
                    recieveSocket = null;
                }
            }
        }

        return rx_string;
    }
}

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

    private UDPReciever PacketReciever;
    private JSONParser parser;

    // Lock that protects the parser and allows one user at a time
    private final Object ParserLock = new Object();

    // Reference to RobotState
    private RobotState mRobotState = RobotState.getInstance();

    // Exector Thread
    // Packets are processed in this thread!
    private ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(2);

    // Thread protected booleans
    // DO NOT access these variables without locks
    private final Object TargetDetectedLock = new Object();
    private boolean TargetDetected = false;
    private final Object BallDetectedLock = new Object();
    private boolean BallDetected = false;

    public void SetTargetDetected(boolean val) {
        synchronized (TargetDetectedLock) {
            TargetDetected = true;
        }
    }

    public void SetBallDetected(boolean val) {
        synchronized (BallDetectedLock) {
            BallDetected = true;
        }
    }

    public boolean IsBallDetected() {
        synchronized (BallDetectedLock) {
            return BallDetected;
        }
    }

    private VisionManager() {
        // Start a Socket to listen to UDP Packet
        // Each thread pass to a processer which
        PacketReciever = new UDPReciever("127.0.0.1", 5800);
        parser = new JSONParser();

        Thread listenerThread =
                new Thread(
                        new Runnable() {
                            @Override
                            public void run() {
                                try {
                                    while (true) {
                                        ProcessPacket();
                                    }
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            }
                        });

        // Set Name and Start Thread!
        listenerThread.setName("VisionListenerThread");
        listenerThread.setPriority(Thread.MIN_PRIORITY + 2);
        listenerThread.start();
    }

    private void ParsePacket(String packet) {
        try {
            JSONObject CurrentPacket;

            // Appears the Parser Object isn't thread safe
            // Make sure the parse function is only called one at a time
            synchronized (ParserLock) {
                CurrentPacket = (JSONObject) parser.parse(packet);
            }

            // Attempt to form target info object
            // is possible this fails!
            TargetInfo ti = new TargetInfo();
            try {
                ti.SetX(((Number) CurrentPacket.get("x")).doubleValue());
                ti.SetY(((Number) CurrentPacket.get("y")).doubleValue());
                ti.SetDistance(((Number) CurrentPacket.get("dis")).doubleValue());
                ti.SetYaw(((Number) CurrentPacket.get("yaw")).doubleValue());
                // ti.SetCameraID(((Number)CurrentPacket.get("id")).intValue());
                ti.SetTargetID(((Number) CurrentPacket.get("targid")).intValue());

                // If we made it to this point we had all the required keys!
                // Now we need to update RobotState with our new values!
                mRobotState.AddVisionObservation(ti);

            } catch (Exception Targ) {
                // Exception Thrown when Trying to retrieve values from json object
                CurrentPacket.get("Target Serialization Exception: " + Targ.getMessage());
            }

        } catch (ParseException pe) {
            // Exception with the Parser
            System.out.println("Parser Exception: " + pe.getMessage());
        } catch (Exception e) {
            // Other Exception
            System.out.println("Parse Packet Exception: " + e.getMessage());
        }
    }

    /*
        Process every single packet
    */
    private void ProcessPacket() {
        try {
            String PacketResult = PacketReciever.getPacket();
            try {

                // Pass call to a Runnable Object
                // this will execute in parallel
                executor.execute(
                        new Runnable() {
                            public void run() {
                                ParsePacket(PacketResult);
                            }
                        });

            } catch (Exception e) {
                System.out.println("Error: Cannot parse recieved UDP json data: " + e.toString());
                e.printStackTrace();
            }
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
