package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Tracks the state of each LED on a strip. No ball = off, loading = flashing yellow, full =
 * flashing green, acquired = solid green.
 */
public class BallIndicator {
  private static BallIndicator instance;
  private int port; // The PWM port number

  /* Needed to make the LED work */
  private AddressableLED addressableLED;
  private AddressableLEDBuffer ledBuffer;

  /* For coordinating flashing across all lights in the strip */
  private Timer stripTimer;

  public enum State {
    EMPTY,
    LOADING,
    FULL,
    ACQUIRED
  }

  /* For keeping track of LED states */
  private class LED {
    private int ledNumber; // The LED's position in the strip
    private State ledState;
    private boolean isOn;

    private LED(int number, State state) {
      ledNumber = number;
      ledState = state;
      isOn = false;
    }

    /* Updates a LED's output */
    private void updateLight() {
      switch (ledState) {

          /* LED off */
        case EMPTY:
          addressableLED.stop();
          isOn = false;
          break;

          /* Flashing yelllow */
        case LOADING:
          if (isOn == true) {
            ledBuffer.setRGB(ledNumber, 255, 255, 0);
            addressableLED.start();
          } else {
            addressableLED.stop();
          }
          break;

          /* Flashing green */
        case FULL:
          if (isOn == true) {
            ledBuffer.setRGB(ledNumber, 255, 255, 0);
            addressableLED.start();
          } else {
            addressableLED.stop();
          }
          break;

          /* Solid green */
        case ACQUIRED:
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
      updateLight();
    }
  }

  private int length; // Number of LEDs on strip
  private LED[] ledStrip; // For keeping track of individual LED states

  private BallIndicator(int port, int length) {
    this.port = port;
    this.length = length;
    stripTimer = new Timer();
    ledStrip = new LED[this.length];
    addressableLED = new AddressableLED(this.port);
    ledBuffer = new AddressableLEDBuffer(this.length);

    addressableLED.setLength(this.length);
    addressableLED.setData(ledBuffer);
    for (int n = 0; n < this.length; n++) {
      ledStrip[n] = new LED(n, State.EMPTY);
    }
  }

  public static BallIndicator getInstance() {
    if (instance == null) {
      instance = new BallIndicator(0, 5);
    }
    return instance;
  }

  /* Updates the strip's state variables */
  void setStripState(State[] newState) {
    for (int n = 0; n < length; n++) {
      ledStrip[n].ledState = newState[n];
    }
    updateStrip();
  }

  /* Updates each light's output */
  private void updateStrip() {
    for (int n = 0; n < length; n++) {
      ledStrip[n].updateLight();
    }
  }

  /* Run every robot loop. Checks whether to toggle the flashing LEDs */
  public void checkTimer() {
    if (stripTimer.get() >= 250) {
      for (int n = 0; n < length; n++) {
        if (ledStrip[n].ledState == State.LOADING || ledStrip[n].ledState == State.ACQUIRED) {
          ledStrip[n].toggle();
        }
      }
      stripTimer.reset();
    }
  }
}
