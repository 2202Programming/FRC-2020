package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem; // TODO: is this the right import?

public class Limelight_Subsystem implements Subsystem {
    private double x, y, area, target;
    private NetworkTable table;

    public Limelight_Subsystem() {
    }

    public void populateLimelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        x = table.getEntry("tx").getDouble(0.0);
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);
        target = table.getEntry("tv").getDouble(0.0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }

    public boolean hasTarget() {
        return target >= 1.0;
    }

    public NetworkTable getTable() {
        return table;
    }
}