package frc.robot;

//import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * Dashboard
 */
public class Dashboard {

  // create pre-defined dashboard tabs to organize for custom layouts
  public static final ShuffleboardTab preRoundTab = Shuffleboard.getTab("Pre-Round");
  //public static final ShuffleboardTab driveTab = Shuffleboard.getTab("DriveTrain");
  public static final ShuffleboardTab systemsTab = Shuffleboard.getTab("Sys");
  //public static final ShuffleboardTab commandTab = Shuffleboard.getTab("Command");
  //public static final ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  //This is all test stuff
  static SendableChooser<String> stringChooser = new SendableChooser<>();

  public static void configure(RobotContainer rc) {

    // String Chooser
    stringChooser.setDefaultOption("Potato", "you chose poorly");
    stringChooser.addOption("No Potato", "you chose very very well" );

    ShuffleboardLayout layout;
    layout = preRoundTab.getLayout("PreRound Test", BuiltInLayouts.kGrid).withSize(3,3);
    layout.add(stringChooser).withPosition(1, 0).withSize(2,1);

    layout = systemsTab.getLayout("Shooter", BuiltInLayouts.kList).withSize(2,3).withPosition(0, 0);
    rc.intake.addDashboardWidgets(layout);
    layout.add(rc.intake); 

    layout = systemsTab.getLayout("DriveTrain", BuiltInLayouts.kList).withSize(2,3).withPosition(2, 0);
    rc.driveTrain.addDashboardWidgets(layout);
    layout.add(rc.driveTrain);

    layout = systemsTab.getLayout("LIDAR", BuiltInLayouts.kList).withSize(2,3).withPosition(4, 0);
    rc.lidar.addDashboardWidgets(layout);
    layout.add(rc.lidar);

  }

  public static String getAutoChoice() {
    String choice = stringChooser.getSelected();
    System.out.println("\n**************"+ choice + "*********\n");
    return choice;
  }

}


