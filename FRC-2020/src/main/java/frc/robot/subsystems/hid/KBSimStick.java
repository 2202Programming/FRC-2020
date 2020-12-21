package frc.robot.subsystems.hid;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 *   KBSim Gladiator Flight Stick
 * 
 *   Warning this stick has modal control on trigger buttons.
 *   Pinky button switches the numbers, so too does the RED/GREEN mode.
 * 
 *   TriggerMode (R/G) is tracked by a command tied to the Mode(30) button
 *   
 *   
 */

public class KBSimStick extends GenericHID {
    TriggerMode currentMode = TriggerMode.RED;

  @SuppressWarnings({ "MemberName", "PMD.SingularField" })
  public enum Button {
    // Buttons
    FlapUp(1), FlapDown(2),  
    A1(3), A2(4), A3(5), B1(6), B2(7), B3(8), 
    C1(9), C2(10), C3(11), 
    Start(12),
    Eject(13),
    // RED MODE
    TriggerRed(14), PinkyTriggerRed(15), TopTriggerRed(18), PinkyTopTriggerRed(19),
    // GREEN MODE
    TriggerGn(16), PinkyTriggerGn(17), TopTriggerGn(20), PinkyTopTriggerGn(21),
    Mode(30);     //this will also switch trigger modes

    public int value;
    private Button(final int val) {
      value = val;
    }
  }

  public enum Axis {
    kX(0), kY(1), kRot(3), kThrottle(2);

    @SuppressWarnings({ "MemberName", "PMD.SingularField" })
    public final int value;

    Axis(final int value) {
      this.value = value;
    }
  }

  public enum TriggerMode {
    RED(0), GREEN(1); 
    @SuppressWarnings({ "MemberName", "PMD.SingularField" })
    public final int value;

    TriggerMode(final int value) {
      this.value = value;
    }
    
    public String toString() {
      return (value == RED.value) ? "Red" : "Green";
    }
  }

  private void toggleMode() {
    currentMode = (currentMode == TriggerMode.RED) ? TriggerMode.GREEN : TriggerMode.RED;
  }

  public KBSimStick(final int port) {
    super(port);
    super.setOutputs(0);

    // use xboxcontroller as reporting, best fit.
    HAL.report(tResourceType.kResourceType_XboxController, port + 1, 0, "kbsimstick");
  }

  public double getAxis(final Axis axis) {
    return getRawAxis(axis.value);
  }

  /**
   * Hand doesn't make sense for this stick, but the GenericHid requires it.
   * Returning the best value we can.
   */
  @Override
  public double getX(final Hand hand) {
    return getAxis(Axis.kX);
  }

  @Override
  public double getY(final Hand hand) {
    return getAxis(Axis.kY);
  }


  /**
   * There are a ton of buttons on the stick, so just use a general reader.
   * If specific funcs are desired, add them here.
   */

  /**
   * Reads the button's current state.
   * 
   * If the TriggerMode ever gets out of sync, we can use special buttons
   * to "fix" it if this ever becomes an issue.
   * 
   * @param button
   * @return state of button
   */
  public boolean getButton(final Button button) {
    return getRawButton(button.value);
  }

  /**
   * Whether the button was pressed since the last check.
   *
   * @return was button was pressed since last check
   */
  public boolean getButtonPressed(final Button button) {
    return getRawButtonPressed(button.value);
  }

  /**
   * Whether the button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getButtonReleased(final Button button) {
    return getRawButtonReleased(button.value);
  }

  public TriggerMode getTriggerMode() {
    return currentMode;
  }

  public void enableTriggerModeTrack() {
    new JoystickButton(this, Button.Mode.value)
        .whenPressed(new TriggerModeTrackCmd(this));
  }

  class TriggerModeTrackCmd extends InstantCommand {
    KBSimStick kbStick;
    TriggerModeTrackCmd(final KBSimStick kbstick) {
      this.kbStick = kbstick;
    }
    @Override
    public void initialize() { 
      kbStick.toggleMode();
      System.out.println("Stick now " + kbStick.getTriggerMode().toString());
    }
  }
}