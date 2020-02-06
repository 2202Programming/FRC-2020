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
import frc.robot.commands.ControlPanelCMD;
import frc.robot.commands.FSMReaderCmd;
import frc.robot.commands.PositionControlPanelCommand;
import frc.robot.commands.RotateControlPanelCommand;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.shift.ShiftGear;
import frc.robot.subsystems.Color_Subsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.triggers.ControlPanelTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain driveTrain = new Drivetrain();
  private static final Color_Subsystem detector = new Color_Subsystem();
  private final GearShifter gearShifter = new GearShifter();
  public static final XboxController driver = new XboxController(0);
  private final ArcadeDrive arcade = new ArcadeDrive(driveTrain, driver);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(driveTrain, arcade);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driver, 4).whenPressed(() -> new ShiftGear(gearShifter, Gear.HIGH_GEAR));
    new JoystickButton(driver, 1).whenPressed(() -> new ShiftGear(gearShifter, Gear.LOW_GEAR));
    //new ControlPanelTrigger(2).whenActive(new ControlPanelCMD(new FSMReaderCmd(), new PositionControlPanelCommand(detector, final_color), new RotateControlPanelCommand(detector, 3)));
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
