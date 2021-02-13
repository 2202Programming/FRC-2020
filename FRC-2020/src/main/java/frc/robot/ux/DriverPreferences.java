// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ux;

import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.ArcadeVelDriveCmd;
import frc.robot.commands.drive.TankVelDriveCmd;

/** Add your docs here. */
public class DriverPreferences {

  // Drive commands
  TankVelDriveCmd tankDriveCmd;
  ArcadeVelDriveCmd arcadeDriveCmd;
  SendableChooser<CommandBase> driveChoices = new SendableChooser<CommandBase>();

  // Robot Limits
  NetworkTableEntry maxSpeedNTE; // [ft/s]
  NetworkTableEntry maxRotationNTE; // [deg/s]
  // NetworkTableEntry sideAccelNTE; // [ft/s^2] - limit turn rate at speed <not
  // implemented>

  DriverPreferences(ShuffleboardTab tab) {
    RobotContainer rc = RobotContainer.getInstance();

    // Create default commands for driver preference selection
    arcadeDriveCmd = new ArcadeVelDriveCmd(rc.driverControls, rc.driveTrain, rc.driveTrain);
    arcadeDriveCmd.setShiftProfile(DriveTrain.shiftCount, DriveTrain.vShiftLow, DriveTrain.vShiftHigh);
    tankDriveCmd = new TankVelDriveCmd(rc.driverControls, rc.driveTrain);
    
    // add to chooser
    SendableRegistry.setName(driveChoices, "DriverMode");
    driveChoices.setDefaultOption("Arcade", arcadeDriveCmd);
    driveChoices.addOption("Tank", tankDriveCmd);

    // put the chooser on the tab we were given
    tab.getLayout("DriveCmd", BuiltInLayouts.kList).withSize(3, 2)
        .withProperties(Map.of()).add(driveChoices);

    ShuffleboardLayout layout = tab.getLayout("Robot Speeds", BuiltInLayouts.kList).withSize(2, 2);
    maxSpeedNTE = layout.addPersistent("feet-per-sec", DriveTrain.maxFPS)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("Min", 0.0, "Max", 20.0, "Block Increment", 5.0))
      .getEntry();
  
    maxSpeedNTE.addListener(event -> 
      {rc.driveTrain.processDashboard(event); },
      EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    maxRotationNTE = layout.addPersistent("deg-per-sec", DriveTrain.maxRotDPS)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("Min", 30.0, "Max", 180.0, "Block Increment", 15 ))
      .getEntry();

    maxRotationNTE.addListener(event -> 
      {rc.driveTrain.processDashboard(event); },
      EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

		
	}

  // Accessors for getting values
  public SendableChooser<CommandBase> getCommandChooser() { return driveChoices; }
  public double getMaxSpeed() {return maxSpeedNTE.getDouble(DriveTrain.maxFPS);  }
  public double getMaxRotation() {return maxRotationNTE.getDouble(DriveTrain.maxRotDPS);  }

}
