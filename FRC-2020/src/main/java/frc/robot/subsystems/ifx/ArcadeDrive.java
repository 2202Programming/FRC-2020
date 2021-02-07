/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;
import edu.wpi.first.wpilibj2.command.Subsystem;
/**
 * Any arcade drive should support these methods.
 */
public interface ArcadeDrive extends Odometry, Subsystem {
    public void arcadeDrive(double xSpeed, double zRot);
    
}
