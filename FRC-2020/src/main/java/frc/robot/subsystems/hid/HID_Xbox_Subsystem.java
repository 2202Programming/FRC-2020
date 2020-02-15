/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.util.ExpoShaper;


/**
 * HID_Subsystem - Human Input Device
 * 
 * Use this class bind devices to meaningful functions.
 * 
 * Add any needed member functions to the DriverControls interface 
 * and then add to any implementations. This way they can work with
 * different controller setup. It will also make it easy to switch 
 * to a different Joystick.
 * 
 * Shouldn't need to make this subsystem a requirement for any
 * command, just reference it. This class is intended to run in
 * the periodic().  It should be run first by being first on the 
 * list.
 * 
 */
public class HID_Xbox_Subsystem extends SubsystemBase implements DriverControls {
  /**
   * Creates a new HID_Subsystem.
   */
  private final XboxController driver;
  private final XboxController assistant;
  private final XboxController switchBoard;
  // private final XboxController phantom = new XboxController(3);

  //Buttons onStartup - in case you want to do something based on controls 
  // being held at power up or on switchboard.
  int initDriverButtons;
  int initAssistentButtons;
  int initSwitchBoardButtons;

  //Arcade 
  ExpoShaper velShaper;
  ExpoShaper rotShaper;
  //Tank 
  ExpoShaper velLeftShaper;
  ExpoShaper velRightShaper;
  
  double vel, z_rot;
  double vLeft, vRight;

  //invertGain is used to change the controls for driving backwards easily.  
  // A negative value indicates you're driving backwards with forwards controls.
  double invertGain = 1.0;

  public HID_Xbox_Subsystem(final double velExpo, final double rotExpo, final double deadzone) {
    // register the devices
    driver = (XboxController) registerController(Id.Driver, new XboxController(Id.Driver.value));
    assistant = (XboxController) registerController(Id.Assistant, new XboxController(Id.Assistant.value));
    switchBoard = (XboxController) registerController(Id.SwitchBoard, new XboxController(Id.SwitchBoard.value));
    // Driver inputs for acade style in normalized units,
    // left Y-stick throttle
    // right X-stick turn rate
    velShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kLeft));
    rotShaper = new ExpoShaper(rotExpo, () -> (driver.getX(Hand.kRight) * -1.0));

    // Tank drive
    velLeftShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kLeft));
    velRightShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kRight));

    // add some deadzone in normalized coordinates
    rotShaper.setDeadzone(deadzone);
    velShaper.setDeadzone(deadzone);

    // read some values to remove unused warning
    assistant.getX();
    switchBoard.getX();

    // read initial buttons for each device - maybe used for configurions
    initDriverButtons = getButtonsRaw(Id.Driver);
    initAssistentButtons = getButtonsRaw(Id.Assistant);
    initSwitchBoardButtons = getButtonsRaw(Id.SwitchBoard);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, use this to read all
    // needed inputs.
    z_rot = rotShaper.get();
    vel = velShaper.get() * invertGain;

    // tank
    if (isControlInverted()) {
      vLeft = velRightShaper.get() * invertGain;
      vRight =  velLeftShaper.get() * invertGain;
    } else {
      vLeft = velLeftShaper.get() * invertGain;
      vRight = velRightShaper.get() * invertGain;
    }
    // Apply a rotation limit on tank with speed
    double Kv = 1.0;
    double avg = (vRight + vLeft)/2.0;
    double maxDelta = 2.0/(Kv*avg*avg + 1.0);
    double absDelta = Math.abs(vLeft - vRight);
    
    if (absDelta > maxDelta) {
      //equalize the sticks so no rotation
      vLeft = avg;
      vRight = avg;
    }

  }

  @Override
  public double getVelocityX() {
    return 0; // todo:fix for mechanum
  }

  @Override
  public double getVelocityY() {
    return vLeft;
  }

  @Override
  public double getVelocity() {
    return vel;
  }

  @Override
  public double getRotation() {
    return z_rot;
  }

  @Override
  public boolean isNormalized() {
    return true;
  }

  // Tank Drive controls
  @Override
  public double getVelocityLeft() {
    return vLeft;
  }

  @Override
  public double getVelocityRight() {
    return vRight;
  }

  @Override
  public void setInvertControls(final boolean invert) {
    invertGain = (invert) ? -1.0 : 1.0;
  }

  @Override
  public boolean isControlInverted() {
    if (invertGain < 0)
      return true;
    return false;
  }

  @Override
  public int getInitialButtons(final Id id) {
    switch (id) 
    {
      case Driver:
        return initSwitchBoardButtons;
      case Assistant:
        return initAssistentButtons;
      case SwitchBoard:
        return initSwitchBoardButtons;
      default:
        return 0;
    }
  }

  /**
     * Returns 0.0 if the given value is within the specified range around zero. The
     * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(final double value, double deadband) {
      if (Math.abs(value) > deadband) {
          if (value > 0.0) {
              return (value - deadband) / (1.0 - deadband);
          } else {
              return (value + deadband) / (1.0 - deadband);
          }
      } else {
          return 0.0;
      }
  }


}
