/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class Log_Subsystem extends SubsystemBase {
  /**
   * Creates a new Log_Subsystem.
   */

  private Limelight_Subsystem limelight;
 // private Lidar_Subsystem lidar;
  public Log_Subsystem(Limelight_Subsystem limelight) {
    this.limelight = limelight;
    //this.lidar = lidar;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelight.log(100);

    if(System.nanoTime() % 10 == 0){
      SmartDashboard.putString("Command: ", Robot.command);
      }

    //lidar.printLog();
  }
}
