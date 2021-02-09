// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

//Loads the json pathweaver files into Trajectories

public class AutoPaths {

  // where to look
  final String PathsDir = Filesystem.getDeployDirectory() + File.separator + "paths";
  
  // what we find
  // if we want to pre-read use a map to hold paths
  //Map<String, Trajectory> trajectories = new HashMap<String, Trajectory>();
  SendableChooser<Trajectory> pathChooser = new SendableChooser<>();

  public AutoPaths() {

    pathChooser.addOption("do nothing", null);

    try {
      // fills chooser from directory
      readPaths();         
    } catch (IOException e) {
      e.printStackTrace();
      //OK to continune, it was some IO error, just eat it.
    }
  }

  public SendableChooser<Trajectory> getChooser() { return pathChooser;}

  /**
   * Reads the selected trajectory file and retrns parsed object.
   * Null if "do nothing" is selected.
   * 
   * @return Trajectory selected 
   */
  public Trajectory get() { return  pathChooser.getSelected();
   }

  void readPaths() throws IOException {
    Stream<Path> paths = Files.walk(Paths.get(PathsDir));
    paths.filter(Files::isRegularFile).forEach(this::buildEntry);      
    paths.close();
  }
  
  void buildEntry(Path file) {
    Path fn = file.getName(file.getNameCount()-1);
    String key = fn.toString().split("\\.")[0]; 
    Trajectory traj = loadTrajectory(fn.toString());
    pathChooser.addOption(key, traj);


  }

  Trajectory loadTrajectory(String filename) {
    try {
      Path trajectoryPath = Paths.get(PathsDir + File.separator + filename);
      return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
    }
    return null;
  }
}
