package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTracker;
import frc.robot.vision.TargetInfo;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

// UDP Reciever used in vision manager
class UDPReciever {
  DatagramSocket recieveSocket = null;
  byte[] receiveData = new byte[2048];
  DatagramPacket recievePacket = null;

  /**
   * Constructor for the UDP reciever. Sets up internal memory structures in prep to start listening
   * for packets.
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

  // Reference to RobotTracker
  private RobotTracker mRobotTracker = RobotTracker.getInstance();

  // Exector Thread
  // Packets are processed in this thread!
  private ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(2);
  private ThreadPoolExecutor RobotTrackerExecutor =
      (ThreadPoolExecutor) Executors.newFixedThreadPool(2);

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
                ProcessPacket();
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
      TargetInfo ti;
      try {
        // Parse Target Info
        ti =
            new TargetInfo(
                ((Number) CurrentPacket.get("targid")).intValue(),
                ((Number) CurrentPacket.get("y")).doubleValue(),
                ((Number) CurrentPacket.get("z")).doubleValue(),
                ((Number) CurrentPacket.get("dis")).doubleValue());

        // ti.SetY(((320/2)- .5) - 40);
        // ti.SetZ((240/2)- .5);

        // Reconvert Field information
        ti.CalculateFields();

        // System.out.println("Recieved Packet: z: " + ti.getZ() + "  y: " + ti.getY());

        // Pass to executor to not block up this field
        RobotTrackerExecutor.execute(
            new Runnable() {
              public void run() {
                // If we made it to this point we had all the required keys!
                // Now we need to update RobotState with our new values!
                // mRobotTracker.addVisionUpdate(Timer.getFPGATimestamp(), ti);
                //  mRobotTracker.UpdateTurretError(ti);

                // Call update function based on target

                try {
                  if (ti.IsHighGoal() == true) {
                    mRobotTracker.UpdateTurretVision(Timer.getFPGATimestamp(), ti);
                  } else if (ti.IsBall() == true) {
                    mRobotTracker.UpdateDriveVision(Timer.getFPGATimestamp(), ti);
                  }
                } catch (Exception e) {
                  System.out.println("CATCH EXCEPTION!");
                  System.out.println(e.getMessage());
                  System.out.println("PRINTING STACK TRACE");
                  e.printStackTrace();
                  int stop = 0;
                }
              }
            });

      } catch (Exception Targ) {
        // Exception Thrown when Trying to retrieve values from json object
        // System.out.println("Packet Storing Exception: " + Targ.getMessage());
        return;
      }
    } catch (Exception e) {
      // Other Exception
      System.out.println("Parse Packet Exception: " + e.getMessage());
    }
  }

  /*
      Process every single packet
  */
  private void ProcessPacket() {

    while (true) {
      try {
        String PacketResult = PacketReciever.getPacket();

        // make sure this packet has size
        if (PacketResult.length() == 0) {
          continue; // null packet.. see ya!
        }

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
  }

  public void zeroSensors() {}

  public void UpdateActiveCamera(int value) {
    // mActiveCamera = mCameras.get(value);
  }

  /*
      Test all Sensors in the Subsystem
  */
  public void checkSubsystem() {}

  @Override
  public void stopSubsytem() {}
}
