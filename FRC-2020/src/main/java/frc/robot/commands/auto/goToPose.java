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
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class goToPose extends CommandBase {
  final VelocityDifferentialDrive_Subsystem drive;
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
  public goToPose(VelocityDifferentialDrive_Subsystem drive) {
    this.drive = drive;


    kinematics = this.drive.getDriveKinematics();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPose = drive.getPose();
    endPose = drive.getSavedPose();

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
    System.out.println("Start X:" + startPose.getX());
    System.out.println("Start Y:" + startPose.getY());
    System.out.println("End X:" + endPose.getX());
    System.out.println("End Y:" + endPose.getY());

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        //new Pose2d(startPose.getX(),startPose.getY(),new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            //new Translation2d((startPose.getX()+endPose.getX())/2,(startPose.getY()+endPose.getY())/2)
            //new Translation2d(2, -1)
        ),
        //new Pose2d(endPose.getX(),endPose.getY(),new Rotation2d(0)),
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
