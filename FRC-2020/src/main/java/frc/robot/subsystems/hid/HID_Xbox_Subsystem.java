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
import frc.robot.util.ExpoShaper;

/**
 * HID_Subsystem - Human Input Device
 * 
 * Use this class bind devices to their uses.
 */
public class HID_Xbox_Subsystem extends SubsystemBase implements DriverControls {
  /**
   * Creates a new HID_Subsystem.
   */
  private final XboxController driver = new XboxController(0);
  //private final XboxController assistant = new XboxController(1);
  //private final XboxController switchBoard = new XboxController(2);
  // private final XboxController phantom = new XboxController(3);

  //Arcade 
  ExpoShaper velShaper;
  ExpoShaper rotShaper;

  double vel, z_rot;

  public HID_Xbox_Subsystem() { 
    // Driver inputs for acade style in normalized units, 
    // left Y-stick throttle
    // right X-stick turn rate
    velShaper = new ExpoShaper(0.3, () -> driver.getY(Hand.kLeft)); 
    rotShaper = new ExpoShaper(0.3, () -> driver.getX(Hand.kRight));
    
    //add some deadzone in normalized coordingates
    rotShaper.setDeadzone(0.05);
    velShaper.setDeadzone(0.05);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, use this to read all 
    // needed inputs.
    z_rot = rotShaper.get();
    vel = velShaper.get();
  } 

  @Override
  public double getVelocitX() {
   // not set for XY control 
    return 0;
  }

  @Override
  public double getVelocityY() {
    // not set for XY control 
    return 0;
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

  @Override
  public void bindButtons() {
    
  }
}
