/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;


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
 * XBox stick signs:
 *   Y stick forward is -1.0, backward is 1.0.
 *   X stick left is -1.0, right is 1.0.
 *  
 * Conventions used robot body axis:
 *    Arcade
 *      Y stick forward creates positive velocity, robot moves forward.
 *      X stick left create positive angular velocity, robots rotates counter-clockwise.
 * 
 *    Tank
 *      Y stick forward will be positive creates positive velocity for that side.
 * 
 */
public class HID_Xbox_Subsystem extends MonitoredSubsystemBase implements DriverControls, Logger {
  /**
   * Creates a new HID_Subsystem.
   */
  private final XboxController driver;
  private final XboxController assistant;
  private final SideboardController switchBoard;
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
    switchBoard = (SideboardController) registerController(Id.SwitchBoard, new SideboardController(Id.SwitchBoard.value));

    /**
     * All Joysticks are read and shaped without sign conventions.
     * Sign convention added on periodic based on the type of driver control
     * being used.
     */
    // Driver inputs for acade style in normalized units,
    // left Y-stick throttle (forward negative) right X-stick turn rate
    velShaper = new ExpoShaper(velExpo, () -> driver.getLeftY());
    rotShaper = new ExpoShaper(rotExpo, () -> driver.getRightX()); 

    // Tank drive Left/Right Y-axis used, forward stick is negative 
    velLeftShaper = new ExpoShaper(velExpo,  () -> driver.getLeftY());
    velRightShaper = new ExpoShaper(velExpo, () -> driver.getRightY());

    // XYRot or Swerve Drive
    // Rotation on Left-X axis,  X-Y throttle on Right
    velXShaper = new ExpoShaper(velExpo,  () -> driver.getRightX());   
    velYShaper = new ExpoShaper(velExpo,  () -> driver.getRightY());
    swRotShaper = new ExpoShaper(rotExpo, () -> driver.getLeftX());  

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

    // read initial buttons for each device - maybe used for configurions
    initDriverButtons = getButtonsRaw(Id.Driver);
    initAssistentButtons = getButtonsRaw(Id.Assistant);
    initSwitchBoardButtons = getButtonsRaw(Id.SwitchBoard);
  }

  @Override
  public void monitored_periodic() {
    // This method will be called once per scheduler frame and read all stick inputs
    // for all possible modes.  This is a few extra calcs and could be made modal to
    // only read/shape the stick mode.
    
    //Arcade
    z_rot = -rotShaper.get() * invertGain; //positive turns robot CCW
    vel = -velShaper.get() * invertGain;   //positive moves robot forward

    // tank
    if (isControlInverted()) {
      velLeft = velRightShaper.get();     //cross left/right
      velRight = velLeftShaper.get();     //and remove inversion
    } else {
      velLeft = -velLeftShaper.get();     //invert so forward stick is positive
      velRight = -velRightShaper.get();   //invert so forward stick is positive
    }
    limitTankRotation();

    //XYRot - field axis, pos X away from driver station, pos y to left side of field
    velY = -velXShaper.get();    //invert, so right stick moves robot, right, lowering Y 
    velX = -velYShaper.get();    //invert, so forward stick is positive, increase X
    xyRot = -swRotShaper.get();  //invert, so positive is CCW 
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
