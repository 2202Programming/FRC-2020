/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drive.shift.GearToggleCmd;
import frc.robot.commands.drive.shift.ToggleAutoShiftCmd;
import frc.robot.commands.intake.IntakeToggleCmd;
import frc.robot.commands.intake.MagazineAdjust;
import frc.robot.commands.intake.MagazineToggleCmd;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.ShooterOn;
//import frc.robot.commands.intake.ShooterOnAuto;
//import frc.robot.commands.intake.ToggleIntakeRaised;
//import frc.robot.commands.auto.DriveOffLine;
//import frc.robot.commands.auto.auto_creep_cmd;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_cmd_group;
//import frc.robot.commands.climb.ClimbGroup;
//import frc.robot.commands.climb.RunWinch;
//import frc.robot.commands.climb.SetClimbArmExtension;
//import frc.robot.commands.climb.SetClimbArmRotation;
import frc.robot.commands.drive.ArcadeDriveCmd;
import frc.robot.commands.drive.ArcadeVelDriveCmd;
import frc.robot.commands.drive.InvertDriveControls;
import frc.robot.commands.drive.SwitchDriveMode;
import frc.robot.commands.drive.TankDriveCmd;
//import frc.robot.subsystems.CameraSubsystem;
//import frc.robot.commands.panel.SimpPositionControl;
//import frc.robot.commands.panel.SimpRotateControl;
//import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.Color_Subsystem;
//import frc.robot.subsystems.Control_Panel;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Log_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxAxis;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.subsystems.hid.XboxButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  //public static PrintWriter outputStream;

  //public final Control_Panel panel;
  //public final Color_Subsystem detector;
  //private final ClimberSubsystem climber;

  TankDriveCmd tankDriveCmd;
  ArcadeDriveCmd arcadeDriveCmd;
  ArcadeVelDriveCmd velDriveCmd;

  //taking input from smartdashboard (probably needs to be moved to shooter_on cmd)
  private double rpmUpper_low = SmartDashboard.getNumber("rpm upper low goal", 1000);
  private double rpmLower_low = SmartDashboard.getNumber("rpm lower low goal", 1000);
  private double rpmUpper_high = SmartDashboard.getNumber("rpm upper high goal", 1900);
  private double rpmLower_high = SmartDashboard.getNumber("rpm lower high goal", 1900);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // put driver controls first so its periodic() is called first.
    driverControls = new HID_Xbox_Subsystem(0.3, 0.9, 0.05); // velExpo,rotExpo, deadzone
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter, 14.0, 100.0); // ft/s, deg/sec
    intake = new Intake_Subsystem(); //shooter percent controlled vs. velocity controlled
    limelight = new Limelight_Subsystem();
    limelight.disableLED();
    logSubsystem = new Log_Subsystem(10); // log every 10 frames - 200mS
    lidar = new Lidar_Subsystem(); 
    
    //panel = new Control_Panel();
    //detector = new Color_Subsystem();
    //climber = new ClimberSubsystem();
    //cameraSubsystem = new CameraSubsystem();
    
    // Add anything that has logging requirements
    logSubsystem.add(/*driveTrain, lidar,*/limelight, intake /*,driverControls, panel, detector*/);

    // Create default commads for driver preference
    tankDriveCmd = new TankDriveCmd(driverControls, driveTrain);
    arcadeDriveCmd = new ArcadeDriveCmd(driverControls, driveTrain);

    velDriveCmd = new ArcadeVelDriveCmd(driverControls, driveTrain, driveTrain, 14.0, 100.0); // fps, dps
    velDriveCmd.setShiftProfile(5, 1.5, 6.8); // counts, ft/s, ft/s

    driveTrain.setDefaultCommand(velDriveCmd);

    // Configure the button bindings
    configureButtonBindings(driverControls);
  }

  private void configureButtonBindings(DriverControls dc) {
    // Drivers buttons
    dc.bind(Id.Driver, XboxButton.A).whenPressed(new InvertDriveControls(dc)); 
    dc.bind(Id.Driver, XboxButton.B).whenPressed(new auto_cmd_group(dc, driveTrain, intake, limelight, lidar));
    dc.bind(Id.Driver, XboxButton.X).whenPressed(new toggleLED(limelight));
    dc.bind(Id.Driver, XboxButton.Y).whenPressed(new ToggleAutoShiftCmd(gearShifter));
    dc.bind(Id.Driver, XboxButton.RB).whenPressed(new SwitchDriveMode(driveTrain, velDriveCmd, arcadeDriveCmd));
    dc.bind(Id.Driver, XboxButton.LB).whenPressed(new GearToggleCmd(driveTrain));
   
    // Assistant's buttons
    dc.bind(Id.Assistant, XboxButton.A).whileHeld(new MagazineAdjust(intake, false, 0.0)); 
    dc.bind(Id.Assistant, XboxButton.B).whenHeld(new ReverseIntake(intake, -0.5));
    dc.bind(Id.Assistant, XboxButton.Y).whenPressed(new MagazineAdjust(intake, true, 0.4), true);
    dc.bind(Id.Assistant, XboxButton.X).whenPressed(new IntakeToggleCmd(intake, 0.7, 0.5)); // mag, intake
    dc.bind(Id.Assistant, XboxButton.RB).whenPressed(new MagazineToggleCmd(intake));
    
    dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenHeld(new ShooterOn(intake, rpmUpper_low, rpmUpper_high, rpmLower_low, rpmLower_high, 0.2)); // rpm_low, rpm_high, seconds mag backup
    
    //auto RPM adjustment from limelight area based on calculated trendlines
    /*
      dc.bindJoystick(Id.Assistant, XboxAxis.TRIGGER_LEFT).whenHeld(new ShooterOnAuto(intake, 0.2, -220.0, 3000.0, -220.0, 3000.0, limelight));
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
    return new auto_cmd_group(driverControls, driveTrain, intake, limelight, lidar);
   }

  /**
   * InitTest() called from Robot when test mode is used. Put code here to fire up
   * in test mode.
   */
  public void initTest() {
  }

  public void runTestPeriod() {
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
