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
import frc.robot.commands.test.TestKBSimMode;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.LowerIntake;
import frc.robot.commands.MagazineAdjust;
import frc.robot.commands.RaiseIntake;
import frc.robot.commands.ShooterOn;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.TankDriveCmd;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.GearShifter.Gear;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.util.input.GeneralTrigger;
import frc.robot.util.input.JoystickTrigger;
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
  public final Intake_Subsystem intake;

  // private final ArcadeDrive arcade = new ArcadeDrive(driveTrain, driver);
  // private final AutomaticGearShift autoGearShift = new
  // AutomaticGearShift(driveTrain, gearShifter);

  //Tests to run during test mode
  TestKBSimMode t1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //put driver controls first so its periodic() is called first.
    
    driverControls = new HID_Xbox_Subsystem(0.3, 0.3, 0.05); // velExpo,rotExpo, deadzone
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter, 15000.0, 5.0);
    intake = new Intake_Subsystem();

    //Use basic arcade drive command
    //driveTrain.setDefaultCommand(new ArcadeDriveCmd(driverControls, driveTrain));
    //Use tank drive to make Dustin happy - dpl 2/8/2020
    driveTrain.setDefaultCommand(new TankDriveCmd(driverControls, driveTrain));
  
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

    //These are for test, not the real controls yet. 2-8-20
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.RB.getCode())
        .whenPressed(new ShiftGearCmd(gearShifter, Gear.LOW_GEAR));
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.LB.getCode())
        .whenPressed(new ShiftGearCmd(gearShifter, Gear.HIGH_GEAR));

    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.X.getCode())
      .whenPressed(new ShooterOn(intake));
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.B.getCode())
      .whenPressed(new IntakeOn(intake));

    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.LB.getCode())
      .whenPressed(new LowerIntake(intake));
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.RB.getCode())
      .whenPressed(new RaiseIntake(intake));

    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.Y.getCode())
      .whileHeld(new MagazineAdjust(intake, true));
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.A.getCode())
      .whileHeld(new MagazineAdjust(intake, false));

      
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

  /**
   *   InitTest() called from Robot when test mode is used.
   *   Put code here to fire up in test mode.
   */
  public void initTest() {
    t1 =  new TestKBSimMode();

  }

  public void runTestPeriod() {
    t1.periodic();
  }

  /**
   * Use this to pass the test command to the main {@link Robot} class.
   *
   * @return the command to run in test mode
   */
  public Command getTestCommand() {
    // An ExampleCommand will run in autonomous
    return new CommandBase() {
    };

  }
}
