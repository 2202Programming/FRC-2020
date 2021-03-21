/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.hid.GeneralTrigger;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.JoystickTrigger;
import frc.robot.subsystems.hid.SideboardController;
import frc.robot.subsystems.hid.XboxAxis;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.hid.XboxPOV;

/**
 * 
 * 2020/12/23   DPL  Strong type Axis vs Button. no getCode() needed when binding commands
 */
public interface DriverControls extends Subsystem {

  public enum Id {
    Driver(0), Assistant(1), SwitchBoard(2), Phantom(3);

    public final int value;

    Id(int value) {
      this.value = value;
    }
  }

  public enum DriverMode {
    Arcade(0), Tank(1), XYRot(2);

    public final int value;

    DriverMode(int value) {
      this.value = value;
    }
  }

  // Static vars to help with implementation
  // Id to Controller map
  public HashMap<Id, GenericHID> deviceMap = new HashMap<Id, GenericHID>();

  // mech & field relative
  default public double getVelocityX()  { return 0.0; }
  default public double getVelocityY()  { return 0.0; }
  default public double getXYRotation() { return 0.0; }

  // tank
  default public double getVelocityLeft()  { return 0.0; }
  default public double getVelocityRight() { return 0.0; }

  // arcade drive
  default public double getVelocity() { return 0.0; }
  default public double getRotation() { return 0.0; }

  // physical or normalize (-1.0, 1.0) values
  default public boolean isNormalized(){return true;}

  // Invert controls - defaults doesn't allow inversion
  default public void setInvertControls(boolean invert) {  }
  default public boolean isControlInverted() { return false; }

  // Buttons set at powerup -saved for later use.
  default public int getInitialButtons(Id id) { return 0;}

  /**
   * SideBoard low level access or just fixed configuration for a controls set
   * This is a bit field that must be decoded into something meaningful. Buttons
   * start counting at 1, bits at zero button 1 = bit 0 1/0 button 2 = bit 1 2/0
   * ... button n = bit (n-1) 2^(n-1)/0
   * 
   * Use and/or logic to decode as needed.
   */
  public default int getButtonsRaw(Id id) {
    return DriverStation.getInstance().getStickButtons(id.value);
  }

  /**
   * Register each of the controllers the DriverControls will use. Do this in the
   * constructor of the implementing class.
   * 
   * @see HID_Xbox_Subsystem
   * 
   * @param id  Id.Driver, Id.Assistent, Id.Sideboard
   * @param hid Input device, xbox or other stick
   * @return
   */
  public default GenericHID registerController(Id id, GenericHID hid) {
    deviceMap.put(id, hid);
    return hid;
  }

  /**
   * Use this to bind a controller's button to a command.
   * 
   * example: bindButton(Id.Driver, XboxButton.A).whenPressed(cmd)
   * 
   * @param id     Id.Driver, Id.Assistant, Id.Sideboard
   * @param button XboxButton 
   * @return JoystickButton
   */
   public default JoystickButton bind(Id id, XboxButton button) {
    return bindButton(id, button.getCode());
  }

  public default JoystickTrigger bind(Id id, XboxAxis axis) {
    return (deviceMap.get(id) != null) ? new JoystickTrigger(deviceMap.get(id), axis.getCode(), 0.5) : null;
  }

  public default POVButton bind(Id id, XboxPOV pov) {
    return (deviceMap.get(id) != null) ? new POVButton(deviceMap.get(id), pov.get()) : null;
  }

  public default JoystickButton bind(Id id, SideboardController.SBButton sw) {
    return  bindButton(id, sw.value);
  }

  
  public default GeneralTrigger bind(Id id, XboxButton buttonId1, XboxButton buttonId2) {
    return (deviceMap.get(id) != null) ? new GeneralTrigger(
        () -> deviceMap.get(id).getRawButton(buttonId1.getCode()) && 
              deviceMap.get(id).getRawButton(buttonId2.getCode())) : null;
  }

  /**
   * prefer the strongly typed calls
   * @param id        Driver, Assisteant, Sideboard
   * @param button
   * @return
   */
  default JoystickButton bindButton(Id id, int button) {
    return (deviceMap.get(id) != null) ? new JoystickButton(deviceMap.get(id), button) : null;
  }
}