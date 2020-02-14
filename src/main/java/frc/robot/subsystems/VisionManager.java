package frc.robot.subsystems;

import frc.robot.Logger;
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
  Logger mLogger;

  /**
   * Constructor for the UDP reciever. Sets up internal memory structures in prep to start listening
   * for packets.
   *
   * @param listen_to_addr String of the IP address of the coprocessor (For example, "10.17.36.8")
   * @param listen_on_port integer port number to listen on. Often between 5800 and 5810 per FMS
   *     whitepaper. Must match whatever port the coprocessor is sending information to.
   */
  public UDPReciever(String listen_from_addr_in, int listen_on_port_in) {
    mLogger = new Logger("visionManager");
    try {
      recieveSocket = new DatagramSocket(listen_on_port_in);
      recievePacket = new DatagramPacket(receiveData, receiveData.length);
      recieveSocket.setSoTimeout(10);
    } catch (IOException e) {
      mLogger.log("Error: Cannot set up UDP reciever socket: " + e.getMessage());
      recieveSocket = null;
    }
  }

  /** Listens for all packets on the connection.. passes to the parser in a thread pool */
  public String getPacket() {
    if (recieveSocket != null) {
      try {
        recieveSocket.receive(recievePacket);
        String rx_string = new String(recievePacket.getData(), 0, recievePacket.getLength());
        return rx_string;
      } catch (IOException e) {

      }
    }
    return "";
  }
}

/*
   Manages all vision tracking cameras
   UDP Packets will be streamed back from tracking camera(s) for speed
   Runs in its own thread
*/
public class VisionManager extends Subsystem {
  Logger mLogger;
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

  private VisionManager() {
    mLogger = new Logger("visionManager");

    mLogger.log("Vision Manager Init!");
    mLogger.log("^^$$$^^^");

    // Start a Socket to listen to UDP Packet
    // Each thread pass to a processer which
    PacketReciever = new UDPReciever("127.0.0.1", 5800);
    parser = new JSONParser();

    Thread listenerThread =
        new Thread(
            new Runnable() {
              @Override
              public void run() {
                mLogger.verbose("DEBUG: Listener Thread Process!");
                processPacket();
                mLogger.verbose("DEBUG: end of listener thread loop");
              }
            });

    // Set Name and Start Thread!
    listenerThread.setName("VisionListenerThread");
    listenerThread.setPriority(Thread.MIN_PRIORITY + 2);
    listenerThread.start();
  }

  private void parsePacket(String packet) {
    if (packet.isEmpty()) return; // if the packet has a length of 0, don't parse it

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
        ti.SetY(((Number) CurrentPacket.get("x")).doubleValue());
        ti.SetZ(((Number) CurrentPacket.get("y")).doubleValue());
        ti.SetDistance(((Number) CurrentPacket.get("dis")).doubleValue());
        ti.SetYaw(((Number) CurrentPacket.get("yaw")).doubleValue());
        // ti.SetCameraID(((Number)CurrentPacket.get("id")).intValue());

        // 0 is high goal, 1 is ball
        ti.SetTargetID(((Number) CurrentPacket.get("targid")).intValue());

        // If we made it to this point we had all the required keys!
        // Now we need to update RobotState with our new values!
        mRobotState.AddVisionObservation(ti);

      } catch (Exception Targ) {
        // Exception Thrown when Trying to retrieve values from json object
        mLogger.warn("Packet Storing Exception: " + Targ.getMessage());
        // CurrentPacket.get("Target Serialization Exception: " + Targ.getMessage());
      }

    } catch (ParseException pe) {
      // Exception with the Parser
      mLogger.warn("Parser Exception: " + pe.getMessage());
    } catch (Exception e) {
      // Other Exception
      mLogger.warn("Parse Packet Exception: " + e.getMessage());
    }
  }

  /*
      Process every single packet
  */
  private void processPacket() {

    while (true) {
      try {
        String PacketResult = PacketReciever.getPacket();
        try {

          // Pass call to a Runnable Object
          // this will execute in parallel
          executor.execute(
              new Runnable() {
                public void run() {
                  parsePacket(PacketResult);
                }
              });

        } catch (Exception e) {
          mLogger.warn("Error: Cannot parse recieved UDP json data: " + e.toString());
          e.printStackTrace();
        }
      } catch (Exception e) {

      }
    }
  }

  public void zeroSensors() {}

  public void updateActiveCamera(int value) {
    // mActiveCamera = mCameras.get(value);
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}
}
