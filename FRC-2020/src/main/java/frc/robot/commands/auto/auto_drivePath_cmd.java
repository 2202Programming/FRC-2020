// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.AutoPaths;
import frc.robot.Constants.RamseteProfile;
import frc.robot.subsystems.ifx.VoltageDrive;

/**
 * 
 * It seems like there should be a more direct way to create the command 
 * and schedule it.  This command is really an instant that creates the pathing
 * and then schedules it.  Why the indirection???
 * 
 * TODO:figure this out.
 */

public class auto_drivePath_cmd extends CommandBase {

  private final VoltageDrive m_robotDrive;
  private AutoPaths autoPath;
  Trajectory path;

  public auto_drivePath_cmd(VoltageDrive drive, AutoPaths paths) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = drive;
    autoPath = paths;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //grab the trajectory determined by the AutoPath
    path = autoPath.get();
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(path.getInitialPose());
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command runcommand = getPathCommand();
    runcommand.schedule();
  }

  public Command getPathCommand() {

    DifferentialDriveKinematics kinematics = m_robotDrive.getDriveKinematics();

    RamseteCommand ramseteCommand = new RamseteCommand(
        path,
        m_robotDrive::getPose,
        new RamseteController(RamseteProfile.kRamseteB, RamseteProfile.kRamseteZeta),
        new SimpleMotorFeedforward(RamseteProfile.ksVolts,
                                  RamseteProfile.kvVoltSecondsPerFoot,
                                  RamseteProfile.kaVoltSecondsSquaredPerFoot),
        kinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(RamseteProfile.kPDriveVel, 0, 0),
        new PIDController(RamseteProfile.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );
  

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Command runcommand = RobotContainer.getTeleCommand();
    //runcommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
