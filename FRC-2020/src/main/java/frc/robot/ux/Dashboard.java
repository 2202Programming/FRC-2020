package frc.robot.ux;

//import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Dashboard
 * 
 * Manage all the choosers and smartdashboard tabs.
 * Custom layouts are setup here.
 * 
 * Choosers can be delivered to subystems or commands via access methods
 * on this class.
 * 
 */
public class Dashboard {

  Command m_defaultCommand;

  // create pre-defined dashboard tabs to organize for custom layouts
  ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
  ShuffleboardTab driverTab = Shuffleboard.getTab("DriversChoice");
  ShuffleboardTab systemsTab = Shuffleboard.getTab("Sys");
  //ShuffleboardTab commandTab = Shuffleboard.getTab("Command");
  //ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  // Layouts 
  AutoPaths  m_autopaths;
  DriverPreferences m_drivers;
  

  public Dashboard(RobotContainer rc) {
    ShuffleboardLayout layout;
    ///layout = matchTab.getLayout("Paths", BuiltInLayouts.kGrid).withSize(3,3);
 
    layout = systemsTab.getLayout("Shooter", BuiltInLayouts.kList).withSize(2,3).withPosition(0, 0);
    rc.intake.addDashboardWidgets(layout);
    rc.intake.getMagazine().getMagPositioner().addDashboardWidgets(layout);
    layout.add(rc.intake); 

    layout = systemsTab.getLayout("DriveTrain", BuiltInLayouts.kList).withSize(2,3).withPosition(2, 0);
    rc.driveTrain.addDashboardWidgets(layout);
    layout.add(rc.driveTrain);

    layout = systemsTab.getLayout("LIDAR", BuiltInLayouts.kList).withSize(2,3).withPosition(4, 0);
    rc.lidar.addDashboardWidgets(layout);
    layout.add(rc.lidar);
    
    m_autopaths = new AutoPaths(matchTab);
    m_drivers = new DriverPreferences(driverTab);

     // Shuffleboard runnable Commands - Soft buttons
    //  matchTab.add("Match Ready", new MatchReadyCmd());
    //  matchTab.add("Zero PC", new SetPowerCellCount(0) );
    //  matchTab.add("Three PC", new SetPowerCellCount(3) );
    //  matchTab.add("Calibrate MagAngle", new MagazineCalibrate());
    //  matchTab.add("ResetPose", new ResetPosition(rc.driveTrain, new Pose2d(2.5, 2.5,new Rotation2d(0.0))));
    //  matchTab.add("DrivePath", new auto_drivePath_cmd(rc.driveTrain, m_autopaths.getChooser()));
    //  matchTab.add("MagLow", new MagazineAngle(rc.intake, Constants.ShooterOnCmd.dataLow));
    //  matchTab.add("MagHigh", new MagazineAngle(rc.intake, Constants.ShooterOnCmd.dataHigh));
    //  matchTab.add("ToggleIntakePose", new IntakePosition(rc.intake, IntakePosition.Direction.Toggle));
    //  matchTab.add("GoToPose", new goToPose(rc.driveTrain, rc.state));
    //  matchTab.add("SavePose", new InstantCommand(rc.state::saveRobotState));
    //  matchTab.add("ToggleAutoShoot", new InstantCommand(rc.intake::toggleAutoShootingMode));
    //  matchTab.add("MagOff", new MagazineBeltAdjust(rc.magazine, false, 0.0));
    //  matchTab.add("MagOn", new MagazineBeltAdjust(rc.magazine, true, 0.4));
    //  matchTab.add("IntakeRev", new IntakePower(rc.intake, IntakePower.Power.ReverseOn, 0.5));
    //  matchTab.add("IntakeToggle", new IntakePower(rc.intake, IntakePower.Power.Toggle, 0.5));
    //  matchTab.add("LimelightToggle", new toggleLED(rc.limelight));
    //  matchTab.add("MagLock", new InstantCommand(rc.intake.getMagazine().getMagPositioner()::lock));
    //  matchTab.add("MagZone1", new MagazineAngle(rc.intake, InterstellarSettings.ssZone1));
    //  matchTab.add("MagZone2", new MagazineAngle(rc.intake, InterstellarSettings.ssZone2));
    //  matchTab.add("MagZone3", new MagazineAngle(rc.intake, InterstellarSettings.ssZone3));
    //  matchTab.add("MagZone4", new MagazineAngle(rc.intake, InterstellarSettings.ssZone4));

  }

  public void add(String tabName, String title, SendableChooser<?> chooser) {
      ShuffleboardTab tab = Shuffleboard.getTab(tabName);
      ShuffleboardLayout layout = tab.getLayout(title);
      layout.add(chooser);
  }
  

  public void addAutoCommand(String name, Command autoCmd) {
    m_autopaths.addAutoCommand(name, autoCmd);
  }

  /**
   * Chooser<> get() methods
   * @return
   */
  public SendableChooser<Trajectory> getTrajectoryChooser() {return m_autopaths.getChooser(); }
  public DriverPreferences getDriverPreferences() {return m_drivers;}
  public Trajectory getTrajectory(String trajName) { return m_autopaths.get(trajName); }
  public Command getAutonomousCommand() { return m_autopaths.getAutonomousCommand(); }
  public Command getDefaultCommand()  {return m_defaultCommand;}
  public void setDefaultCommand(Command cmd) { m_defaultCommand = cmd; }
  
}


