package frc.robot.subsystems.util;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.MatchReadyCmd;
import frc.robot.commands.toggleLED;
import frc.robot.commands.auto.auto_drivePath_cmd;
import frc.robot.commands.auto.goToPose;
import frc.robot.commands.drive.ResetPosition;
import frc.robot.commands.intake.IntakePosition;
import frc.robot.commands.intake.IntakePower;
import frc.robot.commands.intake.MagazineAngle;
import frc.robot.commands.intake.MagazineBeltAdjust;
import frc.robot.commands.intake.MagazineCalibrate;
import frc.robot.commands.intake.SetPowerCellCount;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Magazine_Subsystem;
import frc.robot.subsystems.VelocityDifferentialDrive_Subsystem;
import frc.robot.util.misc.StateMemory;
import frc.robot.ux.Dashboard;

public class WebCommands {

  private NetworkTable table;
  
  public WebCommands(VelocityDifferentialDrive_Subsystem driveTrain, Dashboard dashboard, Intake_Subsystem intake, StateMemory state, Magazine_Subsystem magazine, Limelight_Subsystem limelight){

    table = NetworkTableInstance.getDefault().getTable("Commands");

    ListenerCmdOnTrue("runMatchReady", new MatchReadyCmd() );
    ListenerCmdOnTrue("runZeroPC", new SetPowerCellCount(0) );
    ListenerCmdOnTrue("runThreePC", new SetPowerCellCount(3) );
    ListenerCmdOnTrue("runMagCalibrate", new MagazineCalibrate() );
    ListenerCmdOnTrue("ResetPose", new ResetPosition(driveTrain, new Pose2d(2.5, 2.5,new Rotation2d(0.0))));
    ListenerCmdOnTrue("DrivePath", new auto_drivePath_cmd(driveTrain, dashboard.getTrajectoryChooser()));
    ListenerCmdOnTrue("MagLow", new MagazineAngle(intake, Constants.ShooterOnCmd.dataLow));
    ListenerCmdOnTrue("MagHigh", new MagazineAngle(intake, Constants.ShooterOnCmd.dataHigh));
    ListenerCmdOnTrue("ToggleIntakePose", new IntakePosition(intake, IntakePosition.Direction.Toggle));
    ListenerCmdOnTrue("GoToPose", new goToPose(driveTrain, state));
    ListenerCmdOnTrue("SavePose", new InstantCommand(state::saveRobotState));
    ListenerCmdOnTrue("ToggleAutoShoot", new InstantCommand(intake::toggleAutoShootingMode));
    ListenerCmdOnTrue("MagOff", new MagazineBeltAdjust(magazine, false, 0.0));
    ListenerCmdOnTrue("MagOn", new MagazineBeltAdjust(magazine, true, 0.4));
    ListenerCmdOnTrue("IntakeRev", new IntakePower(intake, IntakePower.Power.ReverseOn, 0.5));
    ListenerCmdOnTrue("IntakeToggle", new IntakePower(intake, IntakePower.Power.Toggle, 0.5));
    ListenerCmdOnTrue("LimelightToggle", new toggleLED(limelight));
    ListenerCmdOnTrue("MagLock", new InstantCommand( intake.getMagazine().getMagPositioner()::lock));
    ListenerCmdOnTrue("MagCalibrate", new InstantCommand( intake.getMagazine().getMagPositioner()::calibrate));


  }

  void  ListenerCmdOnTrue(String entryName, Command cmd) {
    //create entry in our table and set initial value to false
    NetworkTableEntry nte = table.getEntry(entryName);
    nte.setBoolean(false);

    // now construct the command listener, lambda called on value changes 
    table.addEntryListener(entryName, (table, key, entry, value, flags)  -> 
      {
        if (value.getBoolean()) {
          System.out.println("***Web Command - Executing " + entryName);
          CommandScheduler.getInstance().schedule(cmd);
          // just ack the scheduling, in perfect world the cmd would handle this 
          // by taking an NTE or entry string
         nte.setBoolean(false);    
        }
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

}