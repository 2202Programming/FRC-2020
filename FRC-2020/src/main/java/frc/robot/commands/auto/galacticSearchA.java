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

public class galacticSearchA extends SequentialCommandGroup {

  public galacticSearchA(VelocityDifferentialDrive_Subsystem drive, Magazine_Subsystem magazine) {
    String start = "paths/SearchAStart.wpilib.json";
    
    try {
      Path startPath = Filesystem.getDeployDirectory().toPath().resolve(start);
      Trajectory startTraj = TrajectoryUtil.fromPathweaverJson(startPath);
      CommandBase cmd = new followTrajectory(drive, startTraj);
      addCommands(cmd);
      while (!cmd.isFinished()) {}
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + start, ex.getStackTrace());
    }

    String choosen;
    if (magazine.getPC() == 0)
      choosen = "paths/SearchABlue.wpilib.json";
    else
      choosen = "paths/SearchARed.wpilib.json";
    
    try {
      Path choosenPath = Filesystem.getDeployDirectory().toPath().resolve(choosen);
      Trajectory choosenTraj = TrajectoryUtil.fromPathweaverJson(choosenPath);
      addCommands (new followTrajectory(drive, choosenTraj));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + choosen, ex.getStackTrace());
    }
  }
}