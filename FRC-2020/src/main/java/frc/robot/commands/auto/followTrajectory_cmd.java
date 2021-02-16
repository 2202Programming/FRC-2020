package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.ifx.VelocityDrive;

public class followTrajectory_cmd extends CommandBase {
  final VelocityDrive drive;
  final SendableChooser<Trajectory> chooser;

  //
  Trajectory trajectory;
  RamseteCommand ramsete;
  RamseteController rsController;
  DifferentialDriveKinematics kinematics;

  // Ramsete constants - todo wire to ux
  double beta = 2.0;
  double zeta = 0.7;

  /** Creates a new followTrajectory. */
  public followTrajectory_cmd(VelocityDrive drive, SendableChooser<Trajectory> chooser) {
    this.drive = drive;
    this.chooser = chooser;

    kinematics = this.drive.getDriveKinematics();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab the trajectory determined by the AutoPath
    trajectory = chooser.getSelected();
    if (trajectory != null) {
      // Reset odometry to the starting pose of the trajectory.
      drive.resetOdometry(trajectory.getInitialPose());

      // construct new Ramsete
      rsController = new RamseteController(this.beta, this.zeta);
      ramsete = new RamseteCommand(trajectory, drive::getPose, // odmetry package in drive
          rsController, // outer loop non-linear controller (follower)
          kinematics, // robot chassis model
          drive::velocityTankDrive // vel output
      /* no further requirements */);

      // initize ramsete
      ramsete.initialize();
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
    if (ramsete != null)
      ramsete.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ramsete != null)
      return ramsete.isFinished();
    return true;
  }
}
