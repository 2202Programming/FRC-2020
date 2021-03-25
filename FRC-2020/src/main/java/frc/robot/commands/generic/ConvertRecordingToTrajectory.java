// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.generic;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.RobotPhysical;
import frc.robot.commands.generic.PositionRecorder.RecordLine;

/** Add your docs here. */
public class ConvertRecordingToTrajectory {
  DifferentialDriveWheelSpeeds speed;
  Trajectory traj;

  Pose2d pose;
  Trajectory.State state;
  ArrayList<Trajectory.State> states;
  ArrayList<RecordLine> lines = new ArrayList<>();
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotPhysical.WheelAxleDistance);
   
  public ConvertRecordingToTrajectory() {

  }

  double prev_vel = 0;
  double prev_time = 0.0;

  public void processRecording() {
    states.clear();
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

  }


  public void inputFile(String fileName) {
      Path path = Paths.get(fileName);
      lines.clear();

      try 
      {
        Stream<String> stream = Files.lines(path);
        stream.skip(1).forEach(s -> lines.add( RecordLine.fromString(s)) );
        stream.close();
      } catch (IOException ex) {
        System.out.println(ex);
      }

  }


}
