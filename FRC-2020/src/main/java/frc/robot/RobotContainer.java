/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriverPrefs;
import frc.robot.Constants.ShooterOnCmd;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_cmd_group;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.commands.auto.goToPose;
import frc.robot.commands.drive.InvertDriveControls;
import frc.robot.commands.drive.ResetPosition;
import frc.robot.commands.drive.shift.GearToggleCmd;
import frc.robot.commands.drive.shift.ToggleAutoShiftCmd;
import frc.robot.commands.generic.CallFunctionCmd;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePosition.Direction;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.IntakePower.Power;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.MagazineBeltAdjust;
import frc.robot.commands.intake.MagazineCaptureCmd;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.test.path.CreateCircle;
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
  public final GearShifter gearShifter;
  public final VelocityDifferentialDrive_Subsystem driveTrain;
  public final Intake_Subsystem intake;
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
    state = new StateMemory(driveTrain, intake);
    
    //panel = new Control_Panel();
    //detector = new Color_Subsystem();
    //climber = new ClimberSubsystem();
    //cameraSubsystem = new CameraSubsystem();

    // default commands
    intake.getMagazine().setDefaultCommand(new MagazineCaptureCmd(intake));   //uses lightgate to load power cells
    
    // Add anything that has logging requirements
    logSubsystem.add(driveTrain, lidar, limelight, intake, pdp /*,driverControls, panel, detector*/);
    
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
    dc.bind(Id.Driver, XboxButton.START).whenPressed(new followTrajectory(driveTrain, dashboard.getTrajectoryChooser()));
    dc.bind(Id.Driver, XboxButton.BACK).whenPressed(new ResetPosition(driveTrain));



    // Assistant's buttons
    dc.bind(Id.Assistant, XboxButton.A).whileHeld(new MagazineBeltAdjust(mag, false, 0.0)); 
    dc.bind(Id.Assistant, XboxButton.B).whenHeld(new IntakePower(intake, Power.ReverseOn, 0.5));
    dc.bind(Id.Assistant, XboxButton.Y).whenPressed(new MagazineBeltAdjust(mag, true, 0.4));
    dc.bind(Id.Assistant, XboxButton.X).whenPressed(new IntakePower(intake, Power.Toggle, 0.5));
    //dc.bind(Id.Assistant, XboxButton.RB).whenPressed(new MagazineRaiseLowerCmd(mag));
    dc.bind(Id.Assistant, XboxButton.LB).whenPressed(new IntakePosition(intake, Direction.Toggle));
    dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenHeld(new Shoot(intake, ShooterOnCmd.dataHigh)); 
    //dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenHeld(new auto_shooting_cmd(intake, ShooterOnCmd.data, driveTrain, limelight, 1.0)); 
    //dc.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whenPressed(new auto_limelightTurnToShoot_cmd(driveTrain, limelight, 1)); 
    

    //Magazine Angle - POV hat
    dc.bind(Id.Assistant, XboxPOV.POV_UP).whileHeld(new MagazineAngle(intake, MagazineAngle.Direction.Up));
    dc.bind(Id.Assistant, XboxPOV.POV_DOWN).whileHeld(new MagazineAngle(intake, MagazineAngle.Direction.Down));
    dc.bind(Id.Assistant, XboxPOV.POV_LEFT).whenPressed(new MagazineAngle(intake, 35.0));
    dc.bind(Id.Assistant, XboxPOV.POV_RIGHT).whenPressed(new MagazineAngle(intake, 42.0));
    //allow a manual lock on the positioner
    dc.bind(Id.Assistant, XboxButton.L3).whenPressed(new InstantCommand( intake.getMagazine().getMagPositioner()::lock));   

    //test
    CreateCircle circle = new CreateCircle(2, 1.5, -360);
    dc.bind(Id.Assistant, XboxButton.R3).whenPressed(new followTrajectory(driveTrain, circle.getTrajectory()));

    //go from current position to stored position
    dc.bind(Id.Driver, XboxPOV.POV_UP).whenPressed(new goToPose(driveTrain,state));

    //store current position
    dc.bind(Id.Driver, XboxPOV.POV_DOWN).whenPressed(new InstantCommand(state::saveRobotState));

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
