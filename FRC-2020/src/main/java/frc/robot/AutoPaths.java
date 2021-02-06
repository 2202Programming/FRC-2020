// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;


//Loads the json pathweaver files into Trajectories

public class AutoPaths {

  public Trajectory BounceSinglePath;
  public Trajectory TurnFortyFiveLeft;

  public AutoPaths() {

    String trajectoryJSON;

      //load BounceSinglePath 
        trajectoryJSON = "paths/BounceSinglePath.wpilib.json";
        try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          BounceSinglePath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
      

            //load TurnFortyFiveLeft 
            trajectoryJSON = "paths/TurnFortyFiveLeft.wpilib.json";
            try {
              Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
              BounceSinglePath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
              DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }     
          }
}
