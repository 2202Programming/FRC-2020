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
        table = NetworkTableInstance.getDefault().getTable("limelight"); //TODO: could this just be declared in the instance field? Or is this necessary to update the values?
        x = table.getEntry("tx").getDouble(0.0);
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);
        target = table.getEntry("tv").getDouble(0.0);
    }

    /**
     * Gets the horizontal offset from the crosshair to the target (-29.8 to 29.8 degrees)
     * @return the horizontal offset, in degrees
     */
    public double getX() {
        return x;
    }

    /**
     * Gets the vertical offset from teh crosshair to the target (-24.85 to 24.85 degrees)
     * @return the vertical offset, in degrees
     */
    public double getY() {
        return y;
    }

    /**
     * Gets the area of the target as a percentage of the total image
     * @return the target's area, as a value between 0 and 100
     */
    public double getArea() {
        return area;
    }

    /**
     * Gets whether or not the limelight has any valid targets
     * @return <code>true</code> if the limelight has at least one valid target, <code>false</code> otherwise
     */
    public boolean hasTarget() {
        return target >= 1.0;
    }

    /**
     * Gets the limelight's network table
     * @return the limelight's network table
     */
    public NetworkTable getTable() {
        return table;
    }
}