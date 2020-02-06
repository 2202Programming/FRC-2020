package frc.robot.util.misc;

// Converts angle relative to horizontal to angle relative to vertical
public enum Angle {
    Back_Perpendicular_Down(-180), Back_Parallel(-80.0), Back_Hatch_Delivery(-70.0), Back_Cargo_Delivery(-60.0), Perpendicular_Up(0.0),
    Cargo_Delivery(60.0), Starting_Hatch_Hunt(80.0), Hatch_Delivery(70.0), Hatch_Pickup(78.0), Parallel(90.0), Perpendicular_Down(180.0);

    private double phi;

    Angle(double phi) {
        this.phi = phi;
    }

    public double getAngle() {
        return phi;
    }
}