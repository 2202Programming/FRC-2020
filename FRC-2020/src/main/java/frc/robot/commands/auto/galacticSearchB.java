package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

public class galacticSearchB extends SequentialCommandGroup {

  public galacticSearchB(VelocityDifferentialDrive_Subsystem drive, Magazine_Subsystem magazine) {
    String start = "paths/SearchBStart.wpilib.json";
    
    try {
      Path startPath = Filesystem.getDeployDirectory().toPath().resolve(start);
      Trajectory startTraj = TrajectoryUtil.fromPathweaverJson(startPath);
      CommandBase cmd = new followPath(drive, startTraj);
      addCommands(cmd);
      while (!cmd.isFinished()) {}
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + start, ex.getStackTrace());
    }

    String choosen;
    if (magazine.getPC() == 0)
      choosen = "paths/SearchBBlue.wpilib.json";
    else
      choosen = "paths/SearchBRed.wpilib.json";
    
    try {
      Path choosenPath = Filesystem.getDeployDirectory().toPath().resolve(choosen);
      Trajectory choosenTraj = TrajectoryUtil.fromPathweaverJson(choosenPath);
      new followPath(drive, choosenTraj);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + choosen, ex.getStackTrace());
    }
  }
}