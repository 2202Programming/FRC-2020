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
import frc.robot.subsystems.ifx.DriverControls;
//import frc.robot.subsystems.ifx.DriverControls.Id;
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
  //private final XboxController switchBoard = new XboxController(2);
  // private final XboxController phantom = new XboxController(3);

  //Arcade 
  ExpoShaper velShaper;
  ExpoShaper rotShaper;
  //Tank 
  ExpoShaper velLeftShaper;
  ExpoShaper velRightShaper;
  
  double vel, z_rot;
  double vLeft, vRight;

  public HID_Xbox_Subsystem(double velExpo, double rotExpo, double deadzone) { 
    //register the devices
    driver = (XboxController)registerController(Id.Driver, new XboxController(0));
    assistant = (XboxController)registerController(Id.Assistant, new XboxController(1));
    
    // Driver inputs for acade style in normalized units, 
    // left Y-stick throttle
    // right X-stick turn rate
    velShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kLeft)); 
    rotShaper = new ExpoShaper(rotExpo, () -> (driver.getX(Hand.kRight)*-1.0));  //TODO:larry -1
    
    //Tank drive
    velLeftShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kLeft)); 
    velRightShaper = new ExpoShaper(velExpo, () -> driver.getY(Hand.kRight));

    //add some deadzone in normalized coordinates
    rotShaper.setDeadzone(deadzone);
    velShaper.setDeadzone(deadzone);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, use this to read all 
    // needed inputs.
    z_rot = rotShaper.get();
    vel = velShaper.get();
    
    //tank
    vLeft = velLeftShaper.get();
    vRight = velRightShaper.get();
  } 

  @Override
  public double getVelocityX() {
    return 0;   //todo:fix for mechanum 
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

  //Tank Drive controls
  @Override
  public double getVelocityLeft() {
    return vLeft;
  }  
  

  @Override
  public double getVelocityRight() {
    return vRight;
  }

}
