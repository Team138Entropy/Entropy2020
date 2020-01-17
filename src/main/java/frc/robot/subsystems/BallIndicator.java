package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;


public class BallIndicator {
    private static BallIndicator instance;

    //Needed for making the LED work
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;

    //The PWM port
    private int port;

    //Number of LEDs on strip
    private int ledCount;

    private enum State {
        EMPTY, LOADING, FULL, ACQUIRED
    }

    //LED class for keeping track of LED state
    private class LED {

        State ledState;

        LED(State state) {
            this.ledState = state;
        }
    }
    
    //Array to put LEDs in to keep track of LED states
    private LED[] ledStrip;

    //For flashing
    private Timer flashTimer;

    private BallIndicator() {
        ledStrip = new LED[] {new LED(State.EMPTY), new LED(State.EMPTY), new LED(State.EMPTY), new LED(State.EMPTY), 
        new LED(State.EMPTY)};
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
    void setStripState(State[] newState) {
        for (int n = 0; n < ledCount; n++) {
            ledStrip[n].ledState = newState[n];
        }
        updateStrip();
    }

    //Updates each light's output
    private void updateStrip() {
        for (int n = 0; n < ledCount; n++) {
            updateLight(n);
        }
    }

    //Updates a light's output
    private void updateLight(int ledNumber) {
        switch (ledStrip[ledNumber].ledState) {
            case EMPTY:
                addressableLED.stop();
                break;

            case LOADING:
                ledBuffer.setRGB(ledNumber, 255, 255, 0);
                addressableLED.start();
                flash();
                break;

            case FULL:
                ledBuffer.setRGB(ledNumber, 0, 200, 0);
                addressableLED.start();
                break;

            case ACQUIRED:
                ledBuffer.setRGB(ledNumber, 0, 200, 0);
                addressableLED.start();
                flash();
                break;
        }
    }

    //
    private void flash() {
        flashTimer = new Timer();
        boolean b = checkTimer();

        flashTimer.start();

        do { 
        } while (!b);
    }

    private boolean checkTimer() {
        if (flashTimer.get() >= 250) {
            return true;
        } else {
            return false;
        }
    }
}