package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Dashboard
 */
public class Dashboard {

  // create pre-defined dashboard tabs to organize for custom layouts
  //public static final ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrain");
  public static final ShuffleboardTab systemsTab = Shuffleboard.getTab("Systems");
  //public static final ShuffleboardTab commandTab = Shuffleboard.getTab("Command");
  //public static final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  public static void configure(RobotContainer rc) {
    var layout = systemsTab.getLayout("Shooter", BuiltInLayouts.kList).withSize(2,4).withPosition(0, 1);
    rc.intake.addDashboardWidgets(layout);
    layout.add(rc.intake); 

    layout = systemsTab.getLayout("DriveTrain", BuiltInLayouts.kList).withSize(2,3).withPosition(0, 2);
    rc.driveTrain.addDashboardWidgets(layout);
    layout.add(rc.driveTrain);

    layout = systemsTab.getLayout("LIDAR", BuiltInLayouts.kList).withSize(2,3).withPosition(0, 3);
    rc.lidar.addDashboardWidgets(layout);
    layout.add(rc.lidar);

  }

}


