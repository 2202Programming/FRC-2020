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
import frc.robot.commands.MatchReadyCmd;
import frc.robot.commands.intake.MagazineCalibrate;
import frc.robot.commands.intake.SetPowerCellCount;

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
     matchTab.add("Match Ready", new MatchReadyCmd());
     matchTab.add("Zero PC", new SetPowerCellCount(0) );
     matchTab.add("Three PC", new SetPowerCellCount(3) );
     matchTab.add("Calibrate MagAngle", new MagazineCalibrate());
    
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
}


