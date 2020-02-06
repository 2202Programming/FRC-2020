package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import frc.robot.commands.drive.DriveWithLimelightToDistanceDegCmd;

public class SmartDashboardListener implements TableEntryListener {

    @Override
    public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
            int flags) {
        // TODO currently only changes angle drive pid for DriveWithLimelightToDistanceDegCmd (change as necessary)
        if (key.equals("P"))
            DriveWithLimelightToDistanceDegCmd.setKap(value.getDouble());
        else if (key.equals("I"))
            DriveWithLimelightToDistanceDegCmd.setKai(value.getDouble());
        else if (key.equals("D"))
            DriveWithLimelightToDistanceDegCmd.setKai(value.getDouble());

    }
    
}