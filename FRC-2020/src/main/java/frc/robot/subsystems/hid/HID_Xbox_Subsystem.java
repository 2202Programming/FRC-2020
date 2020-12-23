/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.Logger;


/**
 * HID_Subsystem - Human Input Device
 * 
 * Use this class bind devices to meaningful functions.
 * 
 * Add any needed member functions to the DriverControls interface and then add
 * to any implementations. This way they can work with different controller
 * setup. It will also make it easy to switch to a different Joystick.
 * 
 * Shouldn't need to make this subsystem a requirement for any command, just
 * reference it. This class is intended to run in the periodic(). It should be
 * run first by being first on the list.
 * 
 */
public class HID_Xbox_Subsystem extends SubsystemBase implements DriverControls, Logger {
  /**
   * Creates a new HID_Subsystem.
   */
  private final XboxController driver;
  private final XboxController assistant;
  private final XboxController switchBoard;
  // private final XboxController phantom = new XboxController(3);

  // Buttons onStartup - in case you want to do something based on controls
  // being held at power up or on switchboard.
  int initDriverButtons;
  int initAssistentButtons;
  int initSwitchBoardButtons;

  boolean limitRotation = true;

  // Arcade
  ExpoShaper velShaper;
  ExpoShaper rotShaper;
  // Tank
  ExpoShaper velLeftShaper;
  ExpoShaper velRightShaper;

  //XYRot / Swerve (field or robot relative)
  ExpoShaper velXShaper;    // left/right  
  ExpoShaper velYShaper;    // forward/backward 
  ExpoShaper swRotShaper;   // rotation for XYRot

  //values updated each frame
  double vel, z_rot;           //arcade
  double velLeft, velRight;    //tank
  double velX,velY, xyRot;     //XTRot

  // invertGain is used to change the controls for driving backwards easily.
  // A negative value indicates you're driving backwards with forwards controls.
  double invertGain = 1.0;

  public HID_Xbox_Subsystem(final double velExpo, final double rotExpo, final double deadzone) {


    // register the devices
    driver = (XboxController) registerController(Id.Driver, new XboxController(Id.Driver.value));
    assistant = (XboxController) registerController(Id.Assistant, new XboxController(Id.Assistant.value));
    switchBoard = (XboxController) registerController(Id.SwitchBoard, new XboxController(Id.SwitchBoard.value));

    // Driver inputs for acade style in normalized units,
    // left Y-stick throttle right X-stick turn rate
    velShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kLeft));
    rotShaper = new ExpoShaper(rotExpo, () -> (driver.getX(Hand.kRight) * -1.0));

    // Tank drive Left/Right Y-axis used
    velLeftShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kLeft));
    velRightShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kRight));

    // XYRot or Swerve Drive
    // Rotation on Left-X axis,  X-Y throttle on Right
    velXShaper = new ExpoShaper(velExpo, () -> driver.getX(Hand.kRight));     
    velXShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kRight));
    swRotShaper = new ExpoShaper(rotExpo, () -> (- driver.getX(Hand.kLeft)));  //inverted rot
    
    // add some deadzone in normalized coordinates
    rotShaper.setDeadzone(deadzone);
    velShaper.setDeadzone(deadzone);

    // add deadzone for tank
    velLeftShaper.setDeadzone(deadzone);
    velRightShaper.setDeadzone(deadzone);

    // deadzone for swerve
    velXShaper.setDeadzone(deadzone);
    velYShaper.setDeadzone(deadzone);
    swRotShaper.setDeadzone(deadzone);

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
    // This method will be called once per scheduler frame and read all stick inputs
    // for all possible modes.  This is a few extra calcs and could be made modal to
    // only read/shape the stick mode.
    
    //Arcade
    z_rot = rotShaper.get();
    vel = velShaper.get() * invertGain;

    // tank
    if (isControlInverted()) {
      velLeft = velRightShaper.get() * invertGain;
      velRight = velLeftShaper.get() * invertGain;
    } else {
      
      velLeft = velLeftShaper.get() * invertGain;
      velRight = velRightShaper.get() * invertGain;
    }
    limitTankRotation();

    //XYRot
    velX = velXShaper.get();
    velY = velYShaper.get();
    xyRot = swRotShaper.get();
  }
  
  public void setLimitRotation(boolean enableLimit) {
    this.limitRotation = enableLimit;
  }

  private void limitTankRotation() {
    if (limitRotation == false) return;

    // Apply a rotation limit on tank based on command
    double Kv = 33.0;
    double avg = (velRight + velLeft) / 2.0;
    double absV = Math.abs(avg);

    double maxDelta = 1.0 / (Kv * absV * absV * absV + 1.0);
    double absDelta = Math.abs(velLeft - velRight);

    // Handle the different quadrents of the stick for tank drive
    if ((velLeft > 0) && (velRight > 0) || (velLeft < 0) && (velRight < 0)) {
      // Commanding forward or reverse, sticks in same direction

      if (absDelta > maxDelta) {
        // equalize the sticks so no rotation
        velLeft = avg;
        velRight = avg;
      }
    }
    else {
      //Sticks in opposite direction
    }
  }

  @Override
  public double getVelocityX() {
    return velX;
  }

  @Override
  public double getVelocityY() {
    return velY;
  }

  @Override
  public double getXYRotation() {
    return xyRot;
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
    return velLeft;
  }

  @Override
  public double getVelocityRight() {
    return velRight;
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
    switch (id) {
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

  public void log() {
    SmartDashboard.putBoolean("Controls Inverted", isControlInverted());
  }
}
