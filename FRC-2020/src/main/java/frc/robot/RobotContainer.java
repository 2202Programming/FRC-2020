/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveWithLidarToDistanceCmd;
import frc.robot.commands.DriveWithLidarToDistanceDegCmd;
import frc.robot.commands.Mechanum_Joystick_Drive_Cmd;
import frc.robot.commands.drive.DriveWithLimelightToDistanceDegCmd;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Mechanum_Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Mechanum_Drivetrain drivetrain = new Mechanum_Drivetrain();
  public static final XboxController driver = new XboxController(0);
  public final Limelight_Subsystem m_limelight_subsystem = new Limelight_Subsystem();
  public Mechanum_Joystick_Drive_Cmd m_mechanumdrive_cmd = new Mechanum_Joystick_Drive_Cmd(drivetrain, driver);
  public Lidar_Subsystem m_lidar_subsystem = new Lidar_Subsystem();
  //public DriveWithLidarToDistanceCmd m_lidardrive = new DriveWithLidarToDistanceCmd(drivetrain,m_lidar_subsystem,10, 0.5);
  public DriveWithLidarToDistanceDegCmd m_lidardrive_cmd = new DriveWithLidarToDistanceDegCmd(drivetrain,m_lidar_subsystem,10,0,0.2);
  public DriveWithLimelightToDistanceDegCmd m_limelight_cmd = new DriveWithLimelightToDistanceDegCmd(drivetrain, m_limelight_subsystem, 10, 0, 0.5);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //CommandScheduler.getInstance().setDefaultCommand(m_lidar_subsystem, m_lidarupdate);
    CommandScheduler.getInstance().setDefaultCommand(drivetrain, m_mechanumdrive_cmd);
    m_limelight_subsystem.disableLED();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton b_button = new JoystickButton(driver, 2);
    b_button.whenHeld(m_limelight_cmd);
    b_button.whenInactive(m_mechanumdrive_cmd);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new CommandBase() {};//m_autoCommand;
  }
}

