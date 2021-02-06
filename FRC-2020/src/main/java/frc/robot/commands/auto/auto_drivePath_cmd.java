// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;


public class auto_drivePath_cmd extends CommandBase {

  private final VelocityDifferentialDrive_Subsystem m_robotDrive;
  private Trajectory trajectory;
  private String pathname;

  public auto_drivePath_cmd(VelocityDifferentialDrive_Subsystem drive, String path) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_robotDrive = drive;
    pathname = path;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    
    //load path
    String trajectoryJSON = "paths/" + pathname + ".wpilib.json";
    trajectory = new Trajectory();
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
      }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command runcommand = getPathCommand();
    runcommand.schedule();
  }

  public Command getPathCommand() {

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
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
