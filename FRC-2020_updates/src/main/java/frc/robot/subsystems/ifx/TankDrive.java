/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;
import edu.wpi.first.wpilibj2.command.Subsystem;
/**
 *  Any subsystem that can handle normalized tank commands
 */
public interface TankDrive extends Subsystem {
    public void tankDrive(double xSpeed, double ySpeed);
}
