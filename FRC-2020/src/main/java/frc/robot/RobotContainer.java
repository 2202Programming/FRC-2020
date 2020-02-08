/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.shift.ShiftGearCmd;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.hid.XboxControllerButtonCode;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final HID_Xbox_Subsystem driverControls;
  public final GearShifter gearShifter;
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  // private final ArcadeDrive arcade = new ArcadeDrive(driveTrain, driver);
  // private final AutomaticGearShift autoGearShift = new
  // AutomaticGearShift(driveTrain, gearShifter);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //put driver controls first so its periodic() is called first.
    driverControls = new HID_Xbox_Subsystem(0.3, 0.3, 0.05); // velExpo,rotExpo, deadzone
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter, 15000.0, 5.0);

    //Use basic arcade drive command
    driveTrain.setDefaultCommand(new ArcadeDriveCmd(driverControls, driveTrain));
  
    // Configure the button bindings
    configureButtonBindings();
    // CommandScheduler.getInstance().setDefaultCommand(driveTrain, arcade);
    // CommandScheduler.getInstance().setDefaultCommand(gearShifter, autoGearShift);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driver, 4).whenPressed(new ThrottledUpShift(driveTrain,
    // gearShifter));

    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.A.getCode())
        .whenPressed(new ShiftGearCmd(gearShifter, Gear.LOW_GEAR));
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.B.getCode())
        .whenPressed(new ShiftGearCmd(gearShifter, Gear.HIGH_GEAR));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
      // The trajectory: This is the trajectory to be followed; accordingly, we pass the command the trajectory we just constructed in our earlier steps.
      exampleTrajectory,
      // The pose supplier: This is a method reference (or lambda) to the drive subsystem method that returns the pose. The RAMSETE controller needs the current pose measurement to determine the required wheel outputs.
      driveTrain::getPose,
      // The RAMSETE controller: This is the RamseteController object (Java, C++) that will perform the path-following computation that translates the current measured pose and trajectory state into a chassis speed setpoint.
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      // The drive feedforward: This is a SimpleMotorFeedforward object (Java, C++) that will automatically perform the correct feedforward calculation with the feedforward gains (kS, kV, and kA) that we obtained from the drive characterization tool.
      new SimpleMotorFeedforward( DriveConstants.ksVolts,
                                  DriveConstants.kvVoltSecondsPerMeter,
                                  DriveConstants.kaVoltSecondsSquaredPerMeter),
      // The drive kinematics: This is the DifferentialDriveKinematics object (Java, C++) that we constructed earlier in our constants file, and will be used to convert chassis speeds to wheel speeds.
      DriveConstants.kDriveKinematics,
      // The wheel speed supplier: This is a method reference (or lambda) to the drive subsystem method that returns the wheel speeds
      driveTrain::getWheelSpeeds,
      // The left-side PIDController: This is the PIDController object (Java, C++) that will track the left-side wheel speed setpoint, using the P gain that we obtained from the drive characterization tool.
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // The right-side PIDController: This is the PIDController object (Java, C++) that will track the right-side wheel speed setpoint, using the P gain that we obtained from the drive characterization tool.
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // The output consumer: This is a method reference (or lambda) to the drive subsystem method that passes the voltage outputs to the drive motors.
      driveTrain::tankDriveVolts,
      // The robot drive: This is the drive subsystem itself, included to ensure the command does not operate on the drive at the same time as any other command that uses the drive.
      driveTrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
