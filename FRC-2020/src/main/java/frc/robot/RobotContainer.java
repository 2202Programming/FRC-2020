/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverPrefs;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_cmd_group;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.commands.drive.InvertDriveControls;
import frc.robot.commands.drive.ResetPosition;
import frc.robot.commands.drive.shift.GearToggleCmd;
import frc.robot.commands.drive.shift.ToggleAutoShiftCmd;
import frc.robot.commands.generic.CallFunctionCmd;
import frc.robot.commands.intake.IntakeToggleCmd;
import frc.robot.commands.intake.MagazineAdjust;
import frc.robot.commands.intake.MagazineCaptureCmd;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.ShooterOn;
import frc.robot.commands.intake.ToggleIntakeRaised;
import frc.robot.commands.test.subsystem.MagazineManualWind_test;
import frc.robot.commands.test.subsystem.MagazineToggleLock_test;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Log_Subsystem;
import frc.robot.subsystems.Pdp_subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.XboxAxis;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.ux.Dashboard;
import frc.robot.ux.DriverPreferences;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static RobotContainer instance;

  //tracks robot's devices for XML parsing
  static Map<String, Object> deviceMap = new HashMap<String, Object>(); 

  // The robot's subsystems and commands are defined here...
  // public final CameraSubsystem cameraSubsystem;
  public final HID_Xbox_Subsystem driverControls;
  public final GearShifter gearShifter;
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  public final Intake_Subsystem intake;
  public final Limelight_Subsystem limelight;
  public final Lidar_Subsystem lidar;
  public final Log_Subsystem logSubsystem;
  public final Dashboard dashboard;
  public final Pdp_subsystem pdp;
  
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.instance = this;
    // put driver controls first so its periodic() is called first.
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone); 
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter); 
    intake = new Intake_Subsystem(); 
    limelight = new Limelight_Subsystem();
    limelight.disableLED();
    logSubsystem = new Log_Subsystem(10); // log every 10 frames - 200mS
    lidar = new Lidar_Subsystem(); 
    pdp = new Pdp_subsystem();
    
    
    //panel = new Control_Panel();
    //detector = new Color_Subsystem();
    //climber = new ClimberSubsystem();
    //cameraSubsystem = new CameraSubsystem();

    // default commands
    var mag = intake.getMagazine();
    mag.setDefaultCommand(new MagazineCaptureCmd(mag));   //uses lightgate to load power cells
    
    // Add anything that has logging requirements
    logSubsystem.add(/*driveTrain, lidar,*/limelight, intake, pdp /*,driverControls, panel, detector*/);
    
    //Add devices to map for XML parsing usage, names must be unique.
    deviceMap.put("lidar", lidar);
    deviceMap.put("driveTrain", driveTrain);
    deviceMap.put("intake", intake);
    deviceMap.put("limelight", limelight);

    //setup the dashboard programatically, creates any choosers, screens
    dashboard = new Dashboard(this);

    // Configure the button bindings
    configureButtonBindings(driverControls);
  }

  private void configureButtonBindings(DriverControls dc) {
    var mag = intake.getMagazine();

    // Drivers buttons
    dc.bind(Id.Driver, XboxButton.A).whenPressed(new InvertDriveControls(dc)); 
    dc.bind(Id.Driver, XboxButton.B).whenPressed(new auto_cmd_group(dc, driveTrain, intake, limelight, lidar));
    dc.bind(Id.Driver, XboxButton.X).whenPressed(new toggleLED(limelight));
    dc.bind(Id.Driver, XboxButton.Y).whenPressed(new ToggleAutoShiftCmd(gearShifter));
    dc.bind(Id.Driver, XboxButton.RB).whenPressed(new CallFunctionCmd(() -> {return -1;} ));  //placeholder, do nothing
    dc.bind(Id.Driver, XboxButton.LB).whenPressed(new GearToggleCmd(gearShifter));

    //auto path testing
    //dc.bind(Id.Driver, XboxButton.START).whenPressed(new auto_drivePath_cmd(driveTrain, dashboard.getPath()));
    dc.bind(Id.Driver, XboxButton.START).whenPressed(new followTrajectory(driveTrain, dashboard.getPath()));
    dc.bind(Id.Driver, XboxButton.BACK).whenPressed(new ResetPosition(driveTrain));

    // Assistant's buttons
    dc.bind(Id.Assistant, XboxButton.A).whileHeld(new MagazineAdjust(mag, false, 0.0), true); 
    dc.bind(Id.Assistant, XboxButton.B).whenHeld(new ReverseIntake(intake, -0.5));
    dc.bind(Id.Assistant, XboxButton.Y).whenPressed(new MagazineAdjust(mag, true, 0.4), true);
    dc.bind(Id.Assistant, XboxButton.X).whenPressed(new IntakeToggleCmd(intake, 0.7, 0.5)); // mag, intake
    //dc.bind(Id.Assistant, XboxButton.RB).whenPressed(new MagazineRaiseLowerCmd(mag));
    dc.bind(Id.Assistant, XboxButton.LB).whenPressed(new ToggleIntakeRaised(intake));
    dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenHeld(new ShooterOn(intake, ShooterOnCmd.data)); 

    //testing
    dc.bind(Id.Assistant, XboxButton.START).whileHeld(new MagazineManualWind_test(intake, 5.0));
    dc.bind(Id.Assistant, XboxButton.BACK).whileHeld(new MagazineManualWind_test(intake, -5.0));
    dc.bind(Id.Assistant, XboxButton.RB).whenPressed(new MagazineToggleLock_test(intake));
  }

  /**
   * getSubystemByName(String systemName)
   * The XML command file may need access to the subsystem objects
   * this is a clean way to get them.
   * 
   * @return Object 
   */
  public static Object getDeviceByName(String name) {
    if (!deviceMap.containsKey(name)) {
      System.out.println("Device not found on Robot:" + name +
          ".\nPlease check the floor for missing part. ");
      return null;
    }
    return deviceMap.get(name);
  }
  
  public static Map<String, Object> getDeviceMap() {return deviceMap;}

  //static accessor to get the robot container, or device map 
  public static RobotContainer getInstance() {return instance; }

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

  public DriverPreferences getDriverPreferences() { 
    if (dashboard != null) {
      return dashboard.getDriverPreferences();
    }
    return null;
  }

}
