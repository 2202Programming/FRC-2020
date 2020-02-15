/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class Log_Subsystem extends SubsystemBase {
  /**
   * Creates a new Log_Subsystem.
   */

  private Limelight_Subsystem limelight;
  private VelocityDifferentialDrive_Subsystem drive;
  private GearShifter gear;
  private int counter;
 // private Lidar_Subsystem lidar;

  public Log_Subsystem(Limelight_Subsystem limelight, VelocityDifferentialDrive_Subsystem drive, GearShifter gear) {
    this.limelight = limelight;
    this.drive = drive;
    //this.lidar = lidar;
    counter = 0;
  }

  private void log(){
    SmartDashboard.putString("Command: ", Robot.command);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (counter == 10) limelight.log();
    if (counter == 30) drive.log();
    if (counter == 50) log();
    if (counter == 70) gear.log();
    
    counter++;
    if (counter==100) counter = 0;

  }
}
