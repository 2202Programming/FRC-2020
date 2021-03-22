package frc.robot.subsystems.util;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.MatchReadyCmd;
import frc.robot.commands.intake.MagazineCalibrate;
import frc.robot.commands.intake.SetPowerCellCount;

public abstract class WebCommands extends SubsystemBase {

  private NetworkTable table;
  private NetworkTableEntry nt_runMatchReady;
  private NetworkTableEntry nt_runZeroPC;
  private NetworkTableEntry nt_runThreePC;
  private NetworkTableEntry nt_runMagCalibrate;

  public WebCommands(){

    table = NetworkTableInstance.getDefault().getTable("Commands");
    nt_runMatchReady = table.getEntry("runMatchReady");
    nt_runZeroPC = table.getEntry("runZeroPC");
    nt_runThreePC = table.getEntry("runThreePC");
    nt_runMagCalibrate = table.getEntry("runMagCalibrate");

    nt_runMatchReady.setBoolean(false);
    nt_runZeroPC.setBoolean(false);
    nt_runThreePC.setBoolean(false);
    nt_runMagCalibrate.setBoolean(false);

    table.addEntryListener("runMatchReady", (table, key, entry, value, flags) -> {
      CommandScheduler.getInstance().schedule(new MatchReadyCmd());
      nt_runMatchReady.setBoolean(false);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    table.addEntryListener("runZeroPC", (table, key, entry, value, flags) -> {
      CommandScheduler.getInstance().schedule(new SetPowerCellCount(0) );
      nt_runZeroPC.setBoolean(false);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    table.addEntryListener("runThreePC", (table, key, entry, value, flags)  -> {
      CommandScheduler.getInstance().schedule(new SetPowerCellCount(3));
      nt_runThreePC.setBoolean(false);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    table.addEntryListener("runMagCalibrate", (table, key, entry, value, flags)  -> {
      CommandScheduler.getInstance().schedule(new MagazineCalibrate());
      nt_runMagCalibrate.setBoolean(false);
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

  }

}