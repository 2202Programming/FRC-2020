// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generic;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.RobotPhysical;
import frc.robot.commands.generic.PositionRecorder.RecordLine;

/** Add your docs here. */
public class ConvertRecordingToTrajectory { 
  final static String defaultDirectoryName="recordings";
  
  
  //DifferentialDriveWheelSpeeds speed;
  Trajectory trajectory;

  ArrayList<Trajectory.State> states;
  ArrayList<RecordLine> lines;
  DifferentialDriveKinematics kinematics;

  String inFileName;
  Path path;
 
  final String outputDir;

  PrintWriter writer;

  public ConvertRecordingToTrajectory() {
    this(defaultDirectoryName);
  }

  public ConvertRecordingToTrajectory(String folderName) {
    kinematics = new DifferentialDriveKinematics(RobotPhysical.WheelAxleDistance);
    lines = new ArrayList<>();
    states = new ArrayList<>();

    trajectory = null;
    inFileName = null;
    outputDir =  Filesystem.getOperatingDirectory().toString() + "/" +folderName;
    
    // make sure we have a directory to save output files in
    new File(outputDir).mkdirs();
  }

  // used to calc dt and vel
  double prev_vel = 0;
  double prev_time = 0.0;

  public void processRecording() {
    states.clear();
    prev_vel = 0.0;
    prev_time = 0.0;
    lines.forEach(r ->
    {
      double time = r.time;
      double dt = time - prev_time;
      var chassis = kinematics.toChassisSpeeds(r.meas_speed);
      double vel = chassis.vxMetersPerSecond; 
      double accel = (dt > 0.001) ? (vel -prev_vel)/dt : 0.0;
      double curv = chassis.omegaRadiansPerSecond / chassis.vxMetersPerSecond;
      states.add(new Trajectory.State(time, vel, accel, r.robot_pose, curv));

      prev_time = time;
      prev_vel = vel;
    });

    // to do - decimate and filter the states

    // construct the trajectory
    trajectory = new Trajectory(states);
  }
  /**
   *  input file to process
   * 
   * @param fileName  
   */
  public void inputFile(String fileName) {
    this.inFileName = fileName;
    this.path = Paths.get(fileName);
    lines.clear();

    try {
        Stream<String> stream = Files.lines(path);
        stream.skip(1).forEach(s -> lines.add( RecordLine.fromString(s)) );
        stream.close();
    } catch (IOException ex) {
        System.out.println(ex);
    }
    processRecording();
  }

  /**
   * Saves the trajectory file in the output folder
   */
  public void saveTrajectory() {

    String root = inFileName.split(".")[0];
    String output = root + ".json";
    try { 
      File f = new File(outputDir, output);
      f.createNewFile();
      writer = new PrintWriter(f); // PrintWriter is buffered
  
      System.out.println("Recording to: " + f.getAbsolutePath());

    } catch (IOException e) {
      e.printStackTrace();
    }
  

  }

  /**
   * 
   * @return  Trajectory converted from recording
   */
  public Trajectory getTrajectory() {
    return trajectory;
  }

}
