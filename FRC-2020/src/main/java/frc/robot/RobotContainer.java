/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.ifx.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.shift.GearToggleCmd;
import frc.robot.commands.test.TestKBSimMode;
import frc.robot.commands.drive.shift.ShiftGearCmd;
import frc.robot.commands.intake.IntakeToggleCmd;
import frc.robot.commands.intake.MagazineAdjust;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.commands.intake.ToggleIntakeRaised;
import frc.robot.commands.auto.DriveOffLine;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_cmd_group;
import frc.robot.commands.auto.auto_creep_area_cmd;
// import frc.robot.commands.test.TestCmd;  
import frc.robot.commands.test.TestKBSimMode;
import frc.robot.commands.auto.auto_creep_cmd;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.InvertDriveControls;
import frc.robot.commands.drive.SwitchDriveMode;
import frc.robot.commands.drive.TankDriveCmd;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Log_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.util.input.GeneralTrigger;
import frc.robot.util.input.JoystickTrigger;
import frc.robot.util.misc.DPadButton;
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
  public final CameraSubsystem cameraSubsystem;
  public final HID_Xbox_Subsystem driverControls;
  public final GearShifter gearShifter;
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  public final Limelight_Subsystem limelight;
  public final Lidar_Subsystem lidar;
  public final Log_Subsystem logSubsystem;

  Command tankDriveCmd;
  Command arcadeDriveCmd;

  // Tests to run during test mode
  TestKBSimMode t1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // put driver controls first so its periodic() is called first.

    cameraSubsystem = new CameraSubsystem();
    driverControls = new HID_Xbox_Subsystem(0.3, 0.3, 0.05); // velExpo,rotExpo, deadzone
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter, 15000.0, 5.0);
    limelight = new Limelight_Subsystem();
    logSubsystem = new Log_Subsystem(5);   // log every 5 frames - 100mS
    
    //Add anything that has logging requirements
    logSubsystem.add(driveTrain, limelight);

    // Create default commads for driver preference
    tankDriveCmd = new TankDriveCmd(driverControls, driveTrain);
    arcadeDriveCmd = new ArcadeDriveCmd(driverControls, driveTrain);
    driveTrain.setDefaultCommand(arcadeDriveCmd);

    // Configure the button bindings
    /// configureButtonBindings();
    DustinsButtons();
    DPLTestButtons();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.Y.getCode())
     * .whileHeld(new MagazineAdjust(intake, true));
     * driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.A.getCode())
     * .whileHeld(new MagazineAdjust(intake, false));
     */

  }

  private void DustinsButtons() {

    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.LB.getCode())
        .whenPressed(new GearToggleCmd(gearShifter));
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.A.getCode())
        .whenPressed(new InvertDriveControls(driverControls));
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.RB.getCode())
        .whenPressed(new SwitchDriveMode(driveTrain, arcadeDriveCmd, tankDriveCmd));

    //driverControls.bindButton(Id.Driver, XboxControllerButtonCode.B.getCode())
    //    .whileHeld(new auto_creep_cmd(driveTrain, limelight, 0, 0.4, 0.2, -1));

    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.B.getCode())
        .whileHeld(new auto_creep_area_cmd(driveTrain, limelight, lidar, 0, 0.4, 0.2, 1.8));
    
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.X.getCode())
        .whenPressed(new toggleLED(limelight));

        /*
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.X.getCode())
        .whenPressed(new IntakeToggleCmd(intake, 0.7, 0.5)); // mag, intake
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.B.getCode())
        .whenHeld(new ReverseIntake(intake, -0.5));
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.LB.getCode())
        .whenPressed(new ToggleIntakeRaised(intake));
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.Y.getCode())
        .whileHeld(new MagazineAdjust(intake, true));
    driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.A.getCode())
        .whileHeld(new MagazineAdjust(intake, false));
    driverControls.bindJoystick(Id.Assistant, XboxControllerButtonCode.TRIGGER_RIGHT.getCode())
        .whenHeld(new ShooterOn(intake, 1200, 0.4)); // rpm, seconds mag backup
*/

  }

  // Derek's testing...
  private void DPLTestButtons() {

    // testing buttion to enable/disable rotation limits
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.START.getCode()).toggleWhenPressed(new CommandBase() {
      @Override
      public void initialize() {
        driverControls.setLimitRotation(true);
      }

      @Override
      public void end(boolean interrupted) {
        driverControls.setLimitRotation(false);
      }
    });

    // current limit testing UP/DOWN on driver Pad
    DPadButton dUp = new DPadButton((XboxController) DriverControls.deviceMap.get(Id.Driver), DPadButton.Direction.UP);
    dUp.toggleWhenPressed(new CommandBase() {
      @Override
      public void initialize() {
        driveTrain.adjustCurrentLimit(1);
      }
    });

    DPadButton dDown = new DPadButton((XboxController) DriverControls.deviceMap.get(Id.Driver),DPadButton.Direction.DOWN);
    dDown.toggleWhenPressed(new CommandBase() {
      @Override
      public void initialize() {
        driveTrain.adjustCurrentLimit(-1);
      }
    });
    /**
     * // Use the DPad to change the motor ramp rate - increasing time will slow
     * down response DPadButton dLeft = new DPadButton((XboxController)
     * DriverControls.deviceMap.get(Id.Driver), DPadButton.Direction.LEFT);
     * dLeft.toggleWhenPressed(new CommandBase() {
     * 
     * @Override public void initialize() { driveTrain.adjustAccelerationLimit(0.1);
     *           } });
     * 
     *           DPadButton dRight = new DPadButton((XboxController)
     *           DriverControls.deviceMap.get(Id.Driver),
     *           DPadButton.Direction.RIGHT); dRight.toggleWhenPressed(new
     *           CommandBase() {
     * @Override public void initialize() {
     *           driveTrain.adjustAccelerationLimit(-0.1); } });
     */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new CommandBase() {};
    return new DriveOffLine(driveTrain);
  }

  /**
   * InitTest() called from Robot when test mode is used. Put code here to fire up
   * in test mode.
   */
  public void initTest() {
    t1 = new TestKBSimMode();

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
