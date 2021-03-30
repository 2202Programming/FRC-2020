// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ux;

import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.followTrajectory;
import frc.robot.commands.drive.ArcadeVelDriveCmd;
import frc.robot.commands.drive.TankVelDriveCmd;
import frc.robot.subsystems.DrivePreferences;


/** Add your docs here. */
public class DriverPreferences {

  NetworkTable table;

  // Drive commands
  TankVelDriveCmd tankDriveCmd;
  ArcadeVelDriveCmd arcadeDriveCmd;
  SendableChooser<CommandBase> driveChoices = new SendableChooser<CommandBase>();


  DriverPreferences(ShuffleboardTab tab) {
    RobotContainer rc = RobotContainer.getInstance();
    table = NetworkTableInstance.getDefault().getTable("DriverPrefs");

    // Create default commands for driver preference selection
    arcadeDriveCmd = new ArcadeVelDriveCmd(rc.driverControls, rc.driveTrain, rc.gearShifter);
    arcadeDriveCmd.setShiftProfile(DriveTrain.shiftCount, DriveTrain.vShiftLow, DriveTrain.vShiftHigh);
    tankDriveCmd = new TankVelDriveCmd(rc.driverControls, rc.driveTrain);
    
    // add to chooser
    SendableRegistry.setName(driveChoices, "DriverMode");
    driveChoices.setDefaultOption("Arcade", arcadeDriveCmd);
    driveChoices.addOption("Tank", tankDriveCmd);

    // put the chooser on the tab we were given
    tab.getLayout("DriveCmd", BuiltInLayouts.kList).withSize(3, 2)
        .withProperties(Map.of()).add(driveChoices);

    // create controls for drive train preferences driver and tracker
    setup("Driver Speeds", DriveTrain.driverPreferences);
    setup("Tracker Speeds", DriveTrain.trackerPreferences);

    createDoubleTableEntry("Ramsete/Beta", followTrajectory.getBeta(), followTrajectory::setBeta);
    createDoubleTableEntry("Ramsete/Zeta", followTrajectory.getZeta(), followTrajectory::setZeta);
  }

  // Accessors for getting values
  public SendableChooser<CommandBase> getCommandChooser() { return driveChoices; }

  /**
   * helper function to create ux for DrivePreference 
   * @param groupName
   * @param pref
   */
  void setup(String groupName, DrivePreferences pref) {
      createDoubleTableEntry(groupName+"/deg-per-sec",  pref.maxRotRate,    pref::setMaxRotation );
      createDoubleTableEntry(groupName+"/feet-per-sec", pref.maxVelocity,   pref::setMaxVelocity );
      createDoubleTableEntry(groupName+"/slew-rate",    pref.slewRateLimit, pref::setSlewRateLimit );
    }
  
  NetworkTableEntry createDoubleTableEntry(String name, double initValue, DoubleConsumer f) {
    NetworkTableEntry nte = table.getEntry(name);
    nte.setDouble(initValue);
    // grab double when it changes
    nte.addListener( event -> 
      {
          f.accept(event.value.getDouble());
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      return nte;
    }

  }