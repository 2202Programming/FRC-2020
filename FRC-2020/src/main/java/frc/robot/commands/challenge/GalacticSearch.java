package frc.robot.commands.challenge;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class GalacticSearch extends SequentialCommandGroup {

  CommandBase endCmd;
  CommandBase startCmd;

  Trajectory startTraj;
  Trajectory blueTraj;
  Trajectory redTraj;

  VelocityDifferentialDrive_Subsystem drive;
  Magazine_Subsystem magazine;

  // Creates a new GalacticSearch object
  public GalacticSearch(VelocityDifferentialDrive_Subsystem drive, Magazine_Subsystem magazine, boolean isA) {
    this.drive = drive;
    this.magazine = magazine;

    String start;
    String blue;
    String red;
    if (isA) {
      start = "paths/SearchAStart.wpilib.json";
      blue = "paths/SearchABlue.wpilib.json";
      red = "paths/SearchARed.wpilib.json";
    } else {
      start = "paths/SearchBStart.wpilib.json";
      blue = "paths/SearchBBlue.wpilib.json";
      red = "paths/SearchBRed.wpilib.json";
    }

    try {
      Path startPath = Filesystem.getDeployDirectory().toPath().resolve(start);
      startTraj = TrajectoryUtil.fromPathweaverJson(startPath);
      Path bluePath = Filesystem.getDeployDirectory().toPath().resolve(blue);
      blueTraj = TrajectoryUtil.fromPathweaverJson(bluePath);
      Path redPath = Filesystem.getDeployDirectory().toPath().resolve(red);
      redTraj = TrajectoryUtil.fromPathweaverJson(redPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startCmd = new followTrajectory(drive, startTraj);
    addCommands(startCmd);
    while (!startCmd.isFinished()) { // waits until end of path before checking for a ball
    }

    Trajectory choosenTraj;
    if (magazine.getPC() == 0)
      choosenTraj = blueTraj;
    else
      choosenTraj = redTraj;
    endCmd = new followTrajectory(drive, choosenTraj);
    addCommands(endCmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (startCmd.isFinished())
      return endCmd.isFinished();
    else
      return false;
  }
}