// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.challenge;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.InterstellarSettings;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.commands.drive.LimeLightTargetCompensator;
import frc.robot.commands.drive.TankVelDriveCmd;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.IntakePower.Power;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.Shoot;
import frc.robot.commands.intake.ShooterWarmUp;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.ifx.VelocityDrive;

/**
 * 
 * Robot starts near the target for this challenge. X = 0.0 is the Target line Y
 * = 7.5 is the centerline of the field where the target is
 * 
 */
public class InterstellarAccuracy extends SequentialCommandGroup {
  // X Y Heading
  final Pose2d StartPose = new Pose2d(2.0, 7.5, new Rotation2d(0.0));
  final Pose2d Zone1Pose = new Pose2d(5.5, 7.5, new Rotation2d(0.0));
  final Pose2d Zone2Pose = new Pose2d(10.0, 7.5, new Rotation2d(0.0));
  final Pose2d Zone3Pose = new Pose2d(15.0, 7.5, new Rotation2d(0.0));
  final Pose2d Zone4Pose = new Pose2d(20.0, 7.5, new Rotation2d(0.0));
  final Pose2d IntroPose = new Pose2d(24.0, 7.5, new Rotation2d(0.0));

  // subsystems - about everything
  final VelocityDrive drive;
  final Intake_Subsystem intake;
  final Magazine_Subsystem magazine;
  final Limelight_Subsystem limelight;
  final TrajectoryConfig config;
  final TrajectoryConfig reverse_config;

  /** Creates a new InterstellarAccuacy. */
  public InterstellarAccuracy(double maxVel, double maxAccel) {
    this.withName("Intrastellar Accuracy");
    RobotContainer rc = RobotContainer.getInstance();
    drive = rc.driveTrain;
    intake = rc.intake;
    limelight = rc.limelight;
    magazine = intake.getMagazine();

    // trajectory parameters
    config = new TrajectoryConfig(maxVel, maxAccel).setKinematics(drive.getDriveKinematics());
    reverse_config = new TrajectoryConfig(maxVel, maxAccel).setKinematics(drive.getDriveKinematics()).setReversed(true);

    Command leg1 = build_leg(StartPose, Zone1Pose, InterstellarSettings.ssZone1, true);
    Command leg2 = build_leg(IntroPose, Zone2Pose, InterstellarSettings.ssZone2, false);
    Command leg3 = build_leg(IntroPose, Zone3Pose, InterstellarSettings.ssZone3, false);
    Command leg4 = build_leg(IntroPose, Zone4Pose, InterstellarSettings.ssZone4, false);
    Command leg5 = build_leg(IntroPose, Zone4Pose, InterstellarSettings.ssZone4, false);

    this.addCommands(new InstantCommand(() -> {
      magazine.setPC(3);
    }), leg1, leg2, leg3, leg4, leg5);

  }

  /**
   * Build one leg of the challenge
   * 
   * @param startpose
   * @param shootpose
   * @param ss        ShooterSetings for angle, vel to shoot with
   * @return
   */
  Command build_leg(Pose2d startpose, Pose2d shootpose, ShooterSettings ss, boolean start) {
    // move where where we are to shoot point
    var cmd = new SequentialCommandGroup();
    var cfg = (start) ? config : reverse_config; // start goes forward, others are reverse on traj #1
    cmd.addCommands(
      // ready the shooter
      new ShooterWarmUp(ss), 
      new MagazineAngle(intake, ss),
      //drive to the shoot position
      new followTrajectory(drive, computeTrajectory(startpose, shootpose, cfg)),
      // Build the Shoot and return to Introduction zone sequence
      new InstantCommand(limelight::enableLED), 
      new ParallelDeadlineGroup(
            new Shoot(ss).withTimeout(6.0),        // deadline on shoot, times out just in case
            new MagazineAngle(intake, ss),         // adjust if not there
            new LimeLightTargetCompensator(),      // never finishes, stopped when group is done.
            new TankVelDriveCmd(drive, 0.0, 0.0)   // never finishes, feeds watchdog, uses LL compensator   
      ),
      
      //head back and get more power cells
      new InstantCommand(limelight::disableLED), 
      new IntakePower(intake, Power.On, 0.5),
      new followTrajectory(drive, computeTrajectory(shootpose, IntroPose, config)),
      new WaitUntilCommand( magazine::isMagFull).withTimeout(10.0));

    return cmd;
  }

  /**
   *  Computes a simple line from start to end
   * 
   * @param start  initial pose
   * @param end    ending pose
   * @return
   */
  Trajectory computeTrajectory(Pose2d start, Pose2d end, TrajectoryConfig cfg) 
  {
    var trajectory = TrajectoryGenerator.generateTrajectory(
        start,
        List.of(
            //new Translation2d((start.getX()+end.getX())/2,(start.getY()+end.getY())/2)
        ),  end,  cfg );
    return trajectory;
  }
}
