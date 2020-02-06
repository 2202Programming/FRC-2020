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
import frc.robot.commands.drive.shift.ShiftGearCmd;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.hid.XboxControllerButtonCode;

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
    return new CommandBase() {
    };
  }
}
