// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.misc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;

/** Add your docs here. */
public class StateMemory {

  private final VelocityDifferentialDrive_Subsystem driveTrain;
  private final Intake_Subsystem intake;
  private Pose2d savedPose;
  private double savedMagAngle;

  private NetworkTable table;
  private NetworkTableEntry nt_savedPoseX;
  private NetworkTableEntry nt_savedPoseY;
  private NetworkTableEntry nt_savedPoseR;

  public StateMemory(VelocityDifferentialDrive_Subsystem driveTrain, Intake_Subsystem intake){
    this.driveTrain = driveTrain;
    this.intake = intake;

    savedPose = driveTrain.getPose();
    savedMagAngle = intake.getMagazine().getAngle();

    table = NetworkTableInstance.getDefault().getTable("Drivetrain");
    nt_savedPoseX = table.getEntry("SavedX");
    nt_savedPoseY = table.getEntry("SavedY");
    nt_savedPoseR = table.getEntry("SavedR");
  }

  public Pose2d getSavedPose(){
    return savedPose;
  }

  public double getSavedMagAngle() 
  {
    return savedMagAngle;
  }

  public void saveRobotState(){
    savedPose = driveTrain.getPose();
    savedMagAngle = intake.getMagazine().getAngle();
    System.out.println("***Saved X:" + savedPose.getX());
    System.out.println("***Saved Y:" + savedPose.getY());
    System.out.println("***Saved Robot Angle:" + savedPose.getRotation().getDegrees());
    System.out.println("***Saved Mag Angle:" + savedMagAngle);
    nt_savedPoseX.setDouble(savedPose.getX());
    nt_savedPoseY.setDouble(savedPose.getY());
    nt_savedPoseR.setDouble(savedPose.getRotation().getDegrees());
  }
}
