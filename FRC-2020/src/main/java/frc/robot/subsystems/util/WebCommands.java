package frc.robot.subsystems.util;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MatchReadyCmd;
import frc.robot.commands.intake.MagazineCalibrate;
import frc.robot.commands.intake.SetPowerCellCount;

public class WebCommands {

  private NetworkTable table;
  
  public WebCommands(){

    table = NetworkTableInstance.getDefault().getTable("Commands");

    ListenerCmdOnTrue("runMatchReady", new MatchReadyCmd() );
    ListenerCmdOnTrue("runZeroPC", new SetPowerCellCount(0) );
    ListenerCmdOnTrue("runThreePC", new SetPowerCellCount(3) );
    ListenerCmdOnTrue("runMagCalibrate", new MagazineCalibrate() );
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
          table.getEntry(entryName).setBoolean(false);
        }
      }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

}