package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class BallIndicator {
    private static BallIndicator instance;

    // The PWM port number
    private int port;

    // Needed for making the LED work
    private AddressableLED addressableLED;
    private AddressableLEDBuffer ledBuffer;

    // For coordinating flashing across all lights in strip
    private Timer stripTimer;

    public enum State {
        EMPTY,
        LOADING,
        FULL,
        ACQUIRED
    }

    // LED class for keeping track of LED states
    private class LED {

        // LED's position in the strip
        private int ledNumber;
        private State ledState;
        private boolean isOn;

        LED(int number, State state, boolean isOn) {
            ledNumber = number;
            ledState = state;
            this.isOn = isOn;
        }

        // Updates a light's output
        private void updateLight() {
            switch (ledState) {
                case EMPTY:
                    addressableLED.stop();
                    isOn = false;
                    break;

                case LOADING:

                    // Sets the LED to yellow
                    ledBuffer.setRGB(ledNumber, 255, 255, 0);
                    addressableLED.start();
                    isOn = true;
                    break;

                case FULL:

                    // Sets the LED to green
                    ledBuffer.setRGB(ledNumber, 0, 200, 0);
                    addressableLED.start();
                    isOn = true;
                    break;

                case ACQUIRED:

                    // Sets the LED to green
                    ledBuffer.setRGB(ledNumber, 0, 200, 0);
                    addressableLED.start();
                    isOn = true;
                    break;
            }
        }
        private void toggle() {
            if (isOn == true) {
                isOn = false;
            } else if (isOn == false) {
                isOn = true;
            }
        }
    }
    
    // Number of LEDs on strip
    private int length;

    // For keeping track of individual LED states
    private LED[] ledStrip;

    private BallIndicator(int port, int length) {
        stripTimer = new Timer();
        this.port = port;
        this.length = length;
        ledStrip = new LED[this.length];
        addressableLED = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(this.length);
        
        addressableLED.setLength(this.length);
        addressableLED.setData(ledBuffer);
        for (int n = 0; n < this.length; n++) {
            ledStrip[n] = new LED(n, State.EMPTY, false);
        }
        
    }

    public static BallIndicator getInstance() {
        if (instance == null) {
            instance = new BallIndicator(0, 5);
        }
        return instance;
    }

    // Updates the strip's state variables
    void setStripState(State[] newState) {
        for (int n = 0; n < length; n++) {
            ledStrip[n].ledState = newState[n];
        }
        updateStrip();
    }

    // Updates each light's output
    private void updateStrip() {
        for (int n = 0; n < length; n++) {
            ledStrip[n].updateLight();
        }
    }

    public void checkTimer() {
        if (stripTimer.get() >= 250) {
            for (int n = 0; n < length; n++) {
                if (ledStrip[n].ledState == State.LOADING || ledStrip[n].ledState == State.ACQUIRED) {
                    ledStrip[n].toggle();
                }
            }
        }
    }
}
