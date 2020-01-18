/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * Add your docs here.
 */
public class Mechanum_Drivetrain implements Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Talon m_frontMotorR = new Talon(1); //Starboard
  Talon m_backMotorR = new Talon(3); //Starboard
  Talon m_frontMotorL = new Talon(2); //Port Front
  Talon m_backMotorL = new Talon(4); //Port Back

  MecanumDrive m_mechanumDrive = new MecanumDrive(m_frontMotorR, m_backMotorR, m_frontMotorL, m_backMotorL);

  public Mechanum_Drivetrain(){
    
  }

  public void driveCartesian(double ySpeed, double xSpeed, double zRotation){
    m_mechanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }


  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
