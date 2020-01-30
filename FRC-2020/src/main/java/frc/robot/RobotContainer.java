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
import frc.robot.commands.drive.MechanumDrive;
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
  //private final Drivetrain driveTrain = new Drivetrain();
  public final Mechanum_Drivetrain drivetrain = new Mechanum_Drivetrain();
  //private final GearShifter gearShifter = new GearShifter();
  public static final XboxController driver = new XboxController(0);
  //private final ArcadeDrive arcade = new ArcadeDrive(driveTrain, driver);
  public MechanumDrive m_mechanumdrive = new MechanumDrive(drivetrain, driver);

  public Limelight_Subsystem limelight = new Limelight_Subsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(drivetrain, m_mechanumdrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  //  new JoystickButton(driver, 4).whenPressed(() -> new ShiftGear(gearShifter, Gear.HIGH_GEAR));
  //  new JoystickButton(driver, 1).whenPressed(() -> new ShiftGear(gearShifter, Gear.LOW_GEAR));
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
