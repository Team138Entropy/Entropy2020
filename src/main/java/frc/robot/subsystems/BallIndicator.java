package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Tracks the state of each LED on a strip. No ball = off, loading = flashing yellow, full =
 * flashing green, acquired = solid green.
 */
public class BallIndicator {
  private static BallIndicator sInstance;
  private int mPort; // The PWM port number

  /* Needed to make the LED work */
  private AddressableLED mAddressableLED;
  private AddressableLEDBuffer mLedBuffer;

  /* For coordinating flashing across all lights in the strip */
  private Timer mStripTimer;

  public enum State {
    EMPTY,
    LOADING,
    FULL,
    ACQUIRED
  }

  /* For keeping track of LED states */
  private class LED {
    private int mLedNumber; // The LED's number on the strip
    private State mLedState;
    private boolean mEnableLED;

    private LED(int number, State state) {
      mLedNumber = number;
      mLedState = state;
      mEnableLED = false;
    }

    /* Updates a LED's output */
    private void updateLight() {
      switch (mLedState) {

          /* LED off */
        case EMPTY:
          mAddressableLED.stop();
          mEnableLED = false;
          break;

          /* Flashing yelllow */
        case LOADING:
          if (mEnableLED == true) {
            mLedBuffer.setRGB(mLedNumber, 255, 255, 0);
            mAddressableLED.start();
          } else {
            mAddressableLED.stop();
          }
          break;

          /* Flashing green */
        case FULL:
          if (mEnableLED == true) {
            mLedBuffer.setRGB(mLedNumber, 255, 255, 0);
            mAddressableLED.start();
          } else {
            mAddressableLED.stop();
          }
          break;

          /* Solid green */
        case ACQUIRED:
          mLedBuffer.setRGB(mLedNumber, 0, 200, 0);
          mAddressableLED.start();
          mEnableLED = true;
          break;
      }
    }

    private void toggle() {
      mEnableLED = !mEnableLED;
      updateLight();
    }
  }

  private int mLength; // Number of LEDs on strip
  private LED[] mLedStrip; // For keeping track of individual LED states

  private BallIndicator(int port, int length) {
    this.mPort = port;
    this.mLength = length;
    mStripTimer = new Timer();
    mLedStrip = new LED[this.mLength];
    mAddressableLED = new AddressableLED(this.mPort);
    mLedBuffer = new AddressableLEDBuffer(this.mLength);

    mAddressableLED.setLength(this.mLength);
    mAddressableLED.setData(mLedBuffer);
    for (int n = 0; n < this.mLength; n++) {
      mLedStrip[n] = new LED(n, State.EMPTY);
    }
  }

  public static BallIndicator getInstance() {
    if (sInstance == null) {
      sInstance = new BallIndicator(0, 5);
    }
    return sInstance;
  }

  /* Updates the strip's state variables */
  void setStripState(State[] newState) {
    for (int n = 0; n < mLength; n++) {
      mLedStrip[n].mLedState = newState[n];
    }
    updateStrip();
  }

  /* Updates each light's output */
  private void updateStrip() {
    for (int n = 0; n < mLength; n++) {
      mLedStrip[n].updateLight();
    }
  }

  /* Run every robot loop. Checks whether to toggle the flashing LEDs */
  public void checkTimer() {
    if (mStripTimer.get() >= 250) {
      for (int n = 0; n < mLength; n++) {
        if (mLedStrip[n].mLedState == State.LOADING || mLedStrip[n].mLedState == State.FULL) {
          mLedStrip[n].toggle();
        }
      }
      mStripTimer.reset();
    }
  }
}
