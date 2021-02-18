// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ux;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

//Loads the json pathweaver files into Trajectories

public class AutoPaths {

  // where to look
  final String PathsDir = Filesystem.getDeployDirectory() + File.separator + "paths";
  
  // what we find
  SendableChooser<Trajectory> pathChooser = new SendableChooser<>();
  Map<String, Trajectory> m_map = new LinkedHashMap<>();

  public AutoPaths(ShuffleboardTab tab) {

    // create a default trajectory that does nothing 
    SendableRegistry.setName(pathChooser, "PathChooser");
    pathChooser.setDefaultOption("do nothing", null);

    try {
      // fills chooser from directory
      readPaths();         
    } catch (IOException e) {
      e.printStackTrace();
      //OK to continune, it was some IO error, just eat it.
    }

    // put the chooser on the tab we were given
    tab.getLayout("AutoPath", BuiltInLayouts.kList).withSize(2, 2).add(pathChooser);
  }

  public SendableChooser<Trajectory> getChooser() { return pathChooser;}

  /**
   * Reads the selected trajectory file and retrns parsed object.
   * Null if "do nothing" is selected.
   * 
   * @return Trajectory selected 
   */
  //public Trajectory get() { return  pathChooser.getSelected();   }

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
    
    // keep a map so we can access any path we need
    m_map.put(key, traj);
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

  public Trajectory get(String trajName) {
    return m_map.get(trajName);
  }

}
