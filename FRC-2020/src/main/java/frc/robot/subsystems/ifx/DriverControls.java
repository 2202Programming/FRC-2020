/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface DriverControls extends Subsystem {

  public enum Id {
    Driver(0),
    Assistent(1),
    SideBoard(2),
    Phantom(3);

    public final int value;

    Id(int value) {
      this.value = value;
    }
  }
  
  public HashMap<Id, GenericHID> deviceMap = new HashMap<Id, GenericHID>();

  //mech, tank
  public double getVelocitX();
  public double getVelocityY();

  //archive drive
  public double getVelocity();
  public double getRotation();
  
  //physical or normalize values
  boolean isNormalized();

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
  public default GenericHID registerController(Id id, GenericHID hid) {
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
  
}