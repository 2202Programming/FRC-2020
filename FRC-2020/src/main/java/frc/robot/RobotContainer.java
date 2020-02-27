/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.shift.GearToggleCmd;

import frc.robot.commands.test.TestKBSimMode;
import frc.robot.commands.intake.IntakeToggleCmd;
import frc.robot.commands.intake.MagazineAdjust;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.commands.intake.ToggleIntakeRaised;
import frc.robot.commands.auto.DriveOffLine;
//import frc.robot.commands.auto.auto_creep_cmd;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_cmd_group;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.ArcadeVelDriveCmd;
import frc.robot.commands.drive.InvertDriveControls;
import frc.robot.commands.drive.SwitchDriveMode;
import frc.robot.commands.drive.TankDriveCmd;
//import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SimpPositionControl;
import frc.robot.commands.SimpRotateControl;
import frc.robot.subsystems.Color_Subsystem;
import frc.robot.subsystems.Control_Panel;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Log_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.hid.DPadButton;
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
  // public final CameraSubsystem cameraSubsystem;
  public final HID_Xbox_Subsystem driverControls;
  public final GearShifter gearShifter;
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  public final Intake_Subsystem intake;
  public final Limelight_Subsystem limelight;
  public final Lidar_Subsystem lidar;
  public final Log_Subsystem logSubsystem;
  public static final Control_Panel panel = new Control_Panel();
  public static final Color_Subsystem detector = new Color_Subsystem();

  TankDriveCmd tankDriveCmd;
  ArcadeDriveCmd arcadeDriveCmd;
  Command velDriveCmd;

  // Tests to run during test mode
  
  TestKBSimMode t1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // put driver controls first so its periodic() is called first.

    // cameraSubsystem = new CameraSubsystem();
    driverControls = new HID_Xbox_Subsystem(0.3, 0.3, 0.03); // velExpo,rotExpo, deadzone
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter, 10.0, 120.0); 
    intake = new Intake_Subsystem();
    limelight = new Limelight_Subsystem();
    logSubsystem = new Log_Subsystem(5); // log every 5 frames - 100mS
    lidar = new Lidar_Subsystem();

    // Add anything that has logging requirements
    logSubsystem.add(driveTrain, limelight, lidar, intake);

    // Create default commads for driver preference
    tankDriveCmd = new TankDriveCmd(driverControls, driveTrain);
    arcadeDriveCmd = new ArcadeDriveCmd(driverControls, driveTrain);
    velDriveCmd = new ArcadeVelDriveCmd(driverControls, driveTrain, driveTrain, 8.0, 100.0); // fps, dps

    driveTrain.setDefaultCommand(velDriveCmd);

    // Configure the button bindings
    configureButtonBindings();
    DPLTestButtons();
    jasonsButtons();
  }

private void jasonsButtons(){
  driverControls.bindButton(Id.Driver, XboxControllerButtonCode.X.getCode())
  .whenPressed(new toggleLED(limelight));

  driverControls.bindButton(Id.Driver, XboxControllerButtonCode.B.getCode())
  .whenPressed(new auto_cmd_group(driverControls, driveTrain, intake, limelight, lidar));
  
}

  private void configureButtonBindings() {
    // Drivers buttons
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.LB.getCode())
        .whenPressed(new GearToggleCmd(driveTrain)); 
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.A.getCode())
        .whenPressed(new InvertDriveControls(driverControls));
    driverControls.bindButton(Id.Driver, XboxControllerButtonCode.RB.getCode())
        .whenPressed(new SwitchDriveMode(driveTrain, arcadeDriveCmd, tankDriveCmd));

    // Assistant's buttons
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

    // driverControls.bindButton(Id.Assistant, XboxControllerButtonCode.X.getCode())
    // .whenPressed(new auto_creep_cmd(driveTrain, limelight, 0, 10, 10, 10));
    driverControls.bindButton(Id.SwitchBoard, XboxControllerButtonCode.A.getCode())
      .whenPressed(new SimpRotateControl(panel));
    driverControls.bindButton(Id.SwitchBoard, XboxControllerButtonCode.B.getCode())
      .whenPressed(new SimpPositionControl(panel, detector));
    driverControls.bindButton(Id.SwitchBoard, XboxControllerButtonCode.Y.getCode())
      .whenPressed(() -> panel.extendArm()).whenReleased(() -> panel.retractArm());
  }

  // Derek's testing...
  private void DPLTestButtons() {

    // testing buttion to enable/disable tankdrive rotation limits
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

    DPadButton dDown = new DPadButton((XboxController) DriverControls.deviceMap.get(Id.Driver),
        DPadButton.Direction.DOWN);
    dDown.toggleWhenPressed(new CommandBase() {
      @Override
      public void initialize() {
        driveTrain.adjustCurrentLimit(-1);
      }
    });

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
