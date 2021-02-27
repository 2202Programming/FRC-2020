package frc.robot.commands.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ifx.VelocityDrive;

public class goToPose extends CommandBase {
  final VelocityDrive drive;
  //
  Pose2d startPose;
  Pose2d endPose;

  RamseteCommand ramsete;
  RamseteController rsController;
  DifferentialDriveKinematics kinematics;
  // Ramsete constants - todo wire to ux
  double beta = .5;   // larger more aggressive convergence
  double zeta = 0.9; //larger more damping 

  /** Creates a new goToPose. */
  public goToPose(VelocityDrive drive, Pose2d startPose, Pose2d endPose) {
    this.drive = drive;
    this.startPose = startPose;
    this.endPose = endPose;

    kinematics = this.drive.getDriveKinematics();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.RamseteProfile.ksVolts,
                                      Constants.RamseteProfile.kvVoltSecondsPerFoot,
                                      Constants.RamseteProfile.kaVoltSecondsSquaredPerFoot),
                                      Constants.RamseteProfile.kDriveKinematics,
            10);

    TrajectoryConfig config =
    new TrajectoryConfig(Constants.RamseteProfile.kMaxSpeedFeetPerSecond,
                         Constants.RamseteProfile.kMaxAccelerationFeetPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.RamseteProfile.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        startPose,
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            //new Translation2d(1, 1),
            //new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        endPose,
        // Pass config
        config
    );
      // construct new Ramsete
      rsController = new RamseteController(this.beta, this.zeta);
      ramsete = new RamseteCommand(exampleTrajectory, drive::getPose, // odmetry package in drive
          rsController, // outer loop non-linear controller (follower)
          kinematics, // robot chassis model
          drive::velocityTankDrive // vel output
      /* no further requirements */);

      // initize ramsete
      ramsete.initialize();
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
