/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.ShooterOnCmd.dataHigh;
import static frc.robot.Constants.ShooterOnCmd.dataLow;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.DriverPrefs;
import frc.robot.Constants.InterstellarSettings;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_cmd_group;
import frc.robot.commands.auto.auto_drivePath_cmd;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.commands.auto.goToPose;
import frc.robot.commands.challenge.GalacticSearch;
import frc.robot.commands.challenge.InterstellarAccuracy;
import frc.robot.commands.drive.GyroHeadingCompensator;
import frc.robot.commands.drive.InvertDriveControls;
import frc.robot.commands.drive.ResetPosition;
import frc.robot.commands.drive.shift.GearToggleCmd;
import frc.robot.commands.drive.shift.ToggleAutoShiftCmd;
import frc.robot.commands.generic.PositionRecorder;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePosition.Direction;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.IntakePower.Power;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.MagazineBeltAdjust;
import frc.robot.commands.intake.MagazineCaptureCmd;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.test.path.CreateCircle;
import frc.robot.commands.test.subsystem.MonitorDrivetrain;
import frc.robot.commands.test.subsystem.VelocityStepTest;
import frc.robot.subsystems.GearShifter;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Lidar_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Log_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.Pdp_subsystem;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SideboardController.SBButton;
import frc.robot.subsystems.hid.XboxAxis;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.hid.XboxPOV;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.ifx.DriverControls.Id;
import frc.robot.util.misc.StateMemory;
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
  public final Sensors_Subsystem sensors;
  public final GearShifter gearShifter;
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  public final Intake_Subsystem intake;
  public final Magazine_Subsystem magazine;
  public final Limelight_Subsystem limelight;
  public final Lidar_Subsystem lidar;
  public final Log_Subsystem logSubsystem;
  public final Dashboard dashboard;
  public final Pdp_subsystem pdp;
  public StateMemory state;
  
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.instance = this;
    
    //order based on desired periodic() calls
    sensors = new Sensors_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone); 
    gearShifter = new GearShifter();
    driveTrain = new VelocityDifferentialDrive_Subsystem(gearShifter); 
    intake = new Intake_Subsystem();
    magazine = intake.getMagazine();
    limelight = new Limelight_Subsystem();
    limelight.disableLED();
    logSubsystem = new Log_Subsystem(10); // log every 10 frames - 200mS
    lidar = new Lidar_Subsystem(); 
    pdp = new Pdp_subsystem();
    state = new StateMemory(driveTrain, intake);
    
    //panel = new Control_Panel();
    //detector = new Color_Subsystem();
    //climber = new ClimberSubsystem();
    //cameraSubsystem = new CameraSubsystem();

    // default commands
    intake.getMagazine().setDefaultCommand(new MagazineCaptureCmd(intake));   //uses lightgate to load power cells
    
    // Add anything that has logging requirements
    logSubsystem.add(driveTrain, lidar, limelight, intake, pdp /*,driverControls, panel, detector*/);
    logSubsystem.setLogModulo(driveTrain, 1); 

    //Add devices to map for XML parsing usage, names must be unique.
    deviceMap.put("lidar", lidar);
    deviceMap.put("driveTrain", driveTrain);
    deviceMap.put("intake", intake);
    deviceMap.put("limelight", limelight);

    //setup the dashboard programatically, creates any choosers, screens
    dashboard = new Dashboard(this);

    // Configure the button bindings
    configureButtonBindings(driverControls);

    // Setup AutoCommands
    //dashboard.addAutoCommand("match", new auto_cmd_group(driverControls, driveTrain, intake, limelight, lidar));
    dashboard.addAutoCommand("GalaticSearch A", new GalacticSearch(driveTrain, true));
    dashboard.addAutoCommand("GalaticSearch B", new GalacticSearch(driveTrain, false));
    dashboard.addAutoCommand("Interstellar", new InterstellarAccuracy(4, 3));
    
    //test commands
    CreateCircle circle = new CreateCircle(3, 2, -360);
    dashboard.addAutoCommand("computed-circle", new followTrajectory(driveTrain, circle.getTrajectory()));
    dashboard.addAutoCommand("velocityStep", 
    new ParallelCommandGroup(
      new MonitorDrivetrain(driveTrain),              // captures data to NT
      new GyroHeadingCompensator(driveTrain),         // drive straight
      new VelocityStepTest(driveTrain, 3.0, 3.0, 4)   // speed ft/s, duration s, repeat
    ));

    PositionRecorder recorder = new PositionRecorder(driveTrain);
    recorder.setIsRunning(false);

     // force drivetrain to look as dashboard settings, fake an event
     driveTrain.processDashboard(null);
  }

  private void configureButtonBindings(DriverControls dc) {
    // Drivers buttons
    dc.bind(Id.Driver, XboxButton.A).whenPressed(new InvertDriveControls(dc)); 
    dc.bind(Id.Driver, XboxButton.B).whenPressed(new auto_cmd_group(dc, driveTrain, intake, limelight, lidar));
    dc.bind(Id.Driver, XboxButton.X).whenPressed(new toggleLED(limelight));
    dc.bind(Id.Driver, XboxButton.Y).whenPressed(new ToggleAutoShiftCmd(gearShifter));
    //dc.bind(Id.Driver, XboxButton.RB).whenPressed(new InstantCommand( () -> {return;} ));  //placeholder, do nothing
    dc.bind(Id.Driver, XboxButton.LB).whenPressed(new GearToggleCmd(gearShifter));

    //go from current position to stored position
    dc.bind(Id.Driver, XboxPOV.POV_UP).whenPressed(new goToPose(driveTrain, state));

    //store current position
    dc.bind(Id.Driver, XboxPOV.POV_DOWN).whenPressed(new InstantCommand(state::saveRobotState));

    //toggle auto shooting mode
    dc.bind(Id.Driver, XboxButton.R3).whenPressed(new InstantCommand(intake::toggleAutoShootingMode));
    

    // Assistant's buttons
    dc.bind(Id.Assistant, XboxButton.A).whileHeld(new MagazineBeltAdjust(magazine, false, 0.0)); 
    dc.bind(Id.Assistant, XboxButton.B).whenHeld(new IntakePower(intake, Power.ReverseOn, 0.5));
    dc.bind(Id.Assistant, XboxButton.Y).whenPressed(new MagazineBeltAdjust(magazine, true, 0.4));
    dc.bind(Id.Assistant, XboxButton.X).whenPressed(new IntakePower(intake, Power.Toggle, 0.5));
    //dc.bind(Id.Assistant, XboxButton.RB).whenPressed();
    dc.bind(Id.Assistant, XboxButton.LB).whenPressed(new IntakePosition(intake, Direction.Toggle));
    dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenHeld(new Shoot()); 
    //dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenHeld(new auto_shooting_cmd(intake, ShooterOnCmd.data, driveTrain, limelight, 1.0)); 
    //dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenPressed(new auto_limelightTurnToShoot_cmd(driveTrain, limelight, 1)); 
    

    //Magazine Angle - POV hat
    dc.bind(Id.Assistant, XboxPOV.POV_UP).whileHeld(new MagazineAngle(intake, MagazineAngle.Direction.Up));
    dc.bind(Id.Assistant, XboxPOV.POV_DOWN).whileHeld(new MagazineAngle(intake, MagazineAngle.Direction.Down));
    dc.bind(Id.Assistant, XboxPOV.POV_LEFT).whenPressed(new MagazineAngle(intake, dataLow)
      //.andThen(new ShooterWarmUp())  //uncomment for fulltime flywheels
    );
    dc.bind(Id.Assistant, XboxPOV.POV_RIGHT).whenPressed(new MagazineAngle(intake, dataHigh)
      //.andThen(new ShooterWarmUp()) //uncomment for fulltime flywheels
    );

    //allow a manual lock on the positioner
    dc.bind(Id.Assistant, XboxButton.L3).whenPressed(new InstantCommand( intake.getMagazine().getMagPositioner()::lock));   
    dc.bind(Id.Assistant, XboxButton.R3).whenPressed(new InstantCommand( intake.getMagazine().getMagPositioner()::calibrate));   

    // Switchboard
    dc.bind(Id.SwitchBoard, SBButton.Sw21).whenPressed(new MagazineAngle(intake, InterstellarSettings.ssZone1));
    dc.bind(Id.SwitchBoard, SBButton.Sw22).whenPressed(new MagazineAngle(intake, InterstellarSettings.ssZone2));
    dc.bind(Id.SwitchBoard, SBButton.Sw23).whenPressed(new MagazineAngle(intake, InterstellarSettings.ssZone3));
    dc.bind(Id.SwitchBoard, SBButton.Sw24).whenPressed(new MagazineAngle(intake, InterstellarSettings.ssZone4));

    //auto path testing
    dc.bind(Id.Driver,  XboxButton.START).whenPressed(new followTrajectory(driveTrain, dashboard.getTrajectoryChooser()));
    dc.bind(Id.Assistant, XboxButton.START).whenPressed(new auto_drivePath_cmd(driveTrain, dashboard.getTrajectoryChooser()));
    dc.bind(Id.Driver,    XboxButton.BACK).whenPressed(new ResetPosition(driveTrain, new Pose2d(2.5, 2.5,new Rotation2d(0.0))));
    dc.bind(Id.Assistant, XboxButton.BACK).whenPressed(new ResetPosition(driveTrain, new Pose2d(2.5, 2.5,new Rotation2d(0.0))));

    //auto path testing on sideboard  row 3
    dc.bind(Id.SwitchBoard, SBButton.Sw32).whenPressed(new followTrajectory(driveTrain, dashboard.getTrajectoryChooser()));
    dc.bind(Id.SwitchBoard, SBButton.Sw33).whenPressed(new auto_drivePath_cmd(driveTrain, dashboard.getTrajectoryChooser()));
    dc.bind(Id.SwitchBoard, SBButton.Sw34).whenPressed(new ResetPosition(driveTrain, new Pose2d(2.5, 2.5,new Rotation2d(0.0))));
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
      return dashboard.getAutonomousCommand();
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
    /**
     * getTrajectory
     * 
     * Any trajectory file in the paths folder is loaded at powerup to save time.
     * This method will re
     * 
     * @param trajName
     * @return loaded trajectory object
     */
  public Trajectory getTrajectory(String trajName) { 
    return dashboard.getTrajectory(trajName); 
  }
}
