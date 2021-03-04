// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test.subsystem;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ifx.VelocityDrive;

public class MonitorDrivetrain extends CommandBase {

  VelocityDrive drive;
  int counter = 0;
  double startTime = 0;

  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry telemetryHeader = NetworkTableInstance.getDefault().getEntry("/robot/telemetry-header");

  // collect and package data here as sequence of doubles
  double[] numberArray = new double[11];
  ArrayList<Double> entries = new ArrayList<Double>();
  
  // make the collected data a big string
  StringBuilder data = new StringBuilder();

  public MonitorDrivetrain(VelocityDrive drive) {
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LiveWindow.disableAllTelemetry();
    startTime = Timer.getFPGATimestamp();
    counter = 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    System.out.println("Robot disabled");
    drive.velocityTankDrive(0.0, 0.0);

    // data processing step
    data.append(entries.toString());
    data.substring(1, data.length() - 1);
    data.append(", ");
    telemetryEntry.setString(data.toString());
    telemetryHeader.setString(headerDescription());
    entries.clear();
    
    System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
    data.delete(0, data.length());
  }

  /**
   * This command shouldn't finish, it gets run along with another command
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    monitor();
  }

  void monitor() {
    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();
    Pose2d pose = drive.getPose();

    double leftPosition = drive.getLeftPos();
    double leftRate = drive.getLeftVel(false);

    double rightPosition = drive.getRightPos();
    double rightRate = drive.getRightVel(false);

    double battery = RobotController.getBatteryVoltage();

    double X = pose.getX();
    double Y = pose.getY();

    
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = drive.getCommandedWheelSpeeds().leftMetersPerSecond;
    numberArray[3] = drive.getCommandedWheelSpeeds().leftMetersPerSecond;
    numberArray[4] = X;
    numberArray[5] = Y;
    numberArray[6] = leftPosition;
    numberArray[7] = rightPosition;
    numberArray[8] = leftRate;
    numberArray[9] = rightRate;
    numberArray[10] = pose.getRotation().getDegrees();

    // Add data to a string that is uploaded to NT
    for (double num : numberArray) {
      entries.add(num);
    }
    counter++;
  }

  String headerDescription() {
    return "time, battry-volt, vel-cmd-left, vel-cmd-right, X-pos, Y-pos, wheel-left-pos, wheel-right-pos, vel-left, vel-right, theta-deg" ;
  }

}
