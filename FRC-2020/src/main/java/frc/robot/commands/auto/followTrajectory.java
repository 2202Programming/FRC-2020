package frc.robot.commands.auto;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DrivePreferences;
import frc.robot.subsystems.ifx.VelocityDrive;

public class followTrajectory extends CommandBase {

  static Trajectory s_currentTrajectory;
  static long s_startTime;
  static boolean closedLoop = true;
  static NetworkTableEntry nt_closedLoop=null;
  
  static {
    initSmartDashboard();
  }

  final VelocityDrive drive;
  DrivePreferences orig_prefs;

  SendableChooser<Trajectory> chooser = null;

  //
  Trajectory trajectory;
  RamseteCommand ramsete;
  RamseteController rsController;
  DifferentialDriveKinematics kinematics;
  long startTime;

  // Ramsete constants - todo wire to ux
  double beta = 0.12; // larger more aggressive convergence [r/ft]^2  2.0 [r/m]^2 --> .18 r/ft; for beta .001 is out of control
  //JR notes - 0.05, 0.1 look similar to 0.18.  2.0 is out of control
  double zeta = 0.8; //larger more damping 0.99, 0.9, 0.8 look similar.  20 is crazy, so is 2

  Pose2d poseTolerance = new Pose2d(.1, .1, Rotation2d.fromDegrees(1.0));

  /** Creates a new followTrajectory. */
  public followTrajectory(VelocityDrive drive, Trajectory trajectory) {
    this.drive = drive;
    this.trajectory = trajectory;

    kinematics = this.drive.getDriveKinematics();
    addRequirements(drive);
  }

  /** Creates a new followTrajectory. */
  public followTrajectory(VelocityDrive drive, SendableChooser<Trajectory> chooser) {
    this.drive = drive;
    this.chooser = chooser;

    kinematics = this.drive.getDriveKinematics();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orig_prefs = drive.getDrivePreferences();
    drive.setDrivePreferences(DriveTrain.trackerPreferences);

    // pull a trajectory from the chooser if possible
    if (chooser != null) {
      trajectory = chooser.getSelected();
    }
    if (trajectory != null) {
      // for monitoring
      s_currentTrajectory = trajectory;

      // Reset odometry to the starting pose of the trajectory.
      drive.resetOdometry(trajectory.getInitialPose());

      // construct new Ramsete
      rsController = new RamseteController(this.beta, this.zeta);
      rsController.setTolerance(poseTolerance);
      ramsete = new RamseteCommand(trajectory, drive::getPose, // odmetry package in drive
          rsController, // outer loop non-linear controller (follower)
          kinematics, // robot chassis model
          drive::velocityTankDrive // vel output
      /* no further requirements */);

      rsController.setEnabled(followTrajectory.closedLoop);

      // initize ramsete
      ramsete.initialize();
      startTime = RobotController.getFPGATime(); 
      s_startTime = startTime;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ramsete != null)
      ramsete.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_currentTrajectory = null;
    drive.setDrivePreferences(orig_prefs);

    if (ramsete != null)
      ramsete.end(interrupted);
    System.out.println("***FollowTrajectory time (ms) = " + (RobotController.getFPGATime() - startTime)/1000.0);
    System.out.println("***Beta = " + beta + ", Zeta = " + zeta);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ramsete != null)
      return ramsete.isFinished();
    return true;
  }

  // for logging in PositionRecorder
  public static long getStartTime() {return s_startTime; }
  public static Trajectory getCurrentTrajectory() { return s_currentTrajectory;}

  public static void setClosedLoop(boolean cl) {
    closedLoop = cl;
  }

  static void initSmartDashboard() {
    if (nt_closedLoop != null) return;
    ShuffleboardTab tab = Shuffleboard.getTab("Position Recorder");
    nt_closedLoop = tab.add("ClosedLoop Traj", closedLoop).withWidget("Toggle Button").getEntry();
    nt_closedLoop.addListener((EntryNotification e)-> setClosedLoop(e.value.getBoolean()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
  }
}
