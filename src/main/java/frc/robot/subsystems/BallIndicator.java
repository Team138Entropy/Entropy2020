package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;


public class BallIndicator {

    //LED class for keeping track of LED state
    private class LED {

        int state;

        LED(int state) {
            this.state = state;
        }
    }

    private static BallIndicator instance;

    //Needed for making the LED work
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;

    //Array to put LEDs in to keep track of LED states
    private LED[] ledStrip;

    //The PWM port
    private int port;

    //Number of LEDs on strip
    private int ledCount;

    private BallIndicator() {
        ledStrip = new LED[] {new LED(0), new LED(0), new LED(0), new LED(0), new LED(0)};
        addressableLED = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(ledCount);
        
        addressableLED.setLength(ledCount);
        addressableLED.setData(ledBuffer);
    }

    public static BallIndicator getInstance() {
        if (instance == null) {
            instance = new BallIndicator();
        }
        return instance;
    }

    //Updates the strip state
    void setStripState(int[] newState) {
        
        for (int n = 0; n < ledCount; n++) {
            ledStrip[n].state = newState[n];
        }
        updateStrip();
    }

    //Updates each light's output
    void updateStrip() {
        for (int n = 0; n < ledCount; n++) {
            updateLight(n);
        }
    }

    //Updates a light's output
    void updateLight(int ledNumber) {
        if (ledStrip[ledNumber].state == 0) {
            addressableLED.stop();
        }
        if (ledStrip[ledNumber].state == 1) {
            ledBuffer.setRGB(ledNumber, 255, 255, 0);
            addressableLED.start();
        }
        if (ledStrip[ledNumber].state == 2) {
            ledBuffer.setRGB(ledNumber, 0, 200, 0);
            addressableLED.start();
        }
        if (ledStrip[ledNumber].state == 3) {
            ledBuffer.setRGB(ledNumber, 0, 200, 0);
            addressableLED.start();
        }
    }

    //
    void flash() {
        
    }
}