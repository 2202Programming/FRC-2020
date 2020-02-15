/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.input.JoystickTrigger;
import edu.wpi.first.wpilibj.DriverStation;

public interface DriverControls extends Subsystem {

  public enum Id {
    Driver(0),
    Assistant(1),
    SwitchBoard(2),
    Phantom(3);

    public final int value;

    Id(int value) {
      this.value = value;
    }
  }
  //Static vars to help with implementation
  //Id to Controller map
  public HashMap<Id, GenericHID> deviceMap = new HashMap<Id, GenericHID>();

  //mech
  public double getVelocityX();
  public double getVelocityY();

  //tank
  public double getVelocityLeft();
  public double getVelocityRight();

  //arcade drive
  public double getVelocity();
  public double getRotation();
  
  //physical or normalize (-1.0, 1.0) values
  public boolean isNormalized();

  //Invert controls is a common need - force it
  public  void setInvertControls(boolean invert);
  public boolean isControlInverted();

  // Buttons set at powerup -saved for later use.
  public int getInitialButtons(Id id);

  /** 
   * SideBoard low level access or just fixed configuration for a controls set
   * This is a bit field that must be decoded into something meaningful.
   * Buttons start counting at 1, bits at zero
   * button 1 = bit 0  1/0
   * button 2 = bit 1  2/0
   * ...
   * button n = bit (n-1)   2^(n-1)/0
   * 
   * Use and/or logic to decode as needed.
  */
  public default int getButtonsRaw(Id id){
    return DriverStation.getInstance().getStickButtons(id.value);
  }


  /**
   * Register each of the controllers the DriverControls will use.
   * Do this in the constructor of the implementing class.
   * 
   * @see HID_Xbox_Subsystem
   * 
   * @param id  Id.Driver, Id.Assistent, Id.Sideboard
   * @param hid Input device, xbox or other stick
   * @return
   */
  public default GenericHID registerController(Id id, GenericHID hid ) {
    deviceMap.put(id, hid);
    return hid;
  }

/**
 * Use this to bind a controller's button to a command.
 * 
 * example:
 *  bindButton(Id.Driver, XboxController.A.getCode()).whenPressed(cmd)
 * 
 * @param id  Id.Driver, Id.Assistent, Id.Sideboard
 * @param button  int that represents the button
 * @return
 */
  public default JoystickButton bindButton(Id id, int button) {
    return (deviceMap.get(id) != null) ?  
        new JoystickButton(deviceMap.get(id) , button) : null;
  }

  public default JoystickTrigger bindJoystick(Id id, int axis) {
    return (deviceMap.get(id) != null) ?  
        new JoystickTrigger(deviceMap.get(id), axis, 0.5) : null;
  }


}