package frc.robot.util.pid;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.EditablePID;

/**
 * A PIDEditor class. Used as a seperate object to be used by subsystems implementing the EditablePID interface.
 * To be tested.
 * 
 * @author Kevin Li
 */
public class PIDEditor { // as a separate object to be used by subsystems
    private EditablePID subsystem;
    private String pKey, iKey, dKey;
    private boolean initialized = false;

    public PIDEditor() {
        // empty constructor to allow for construction by EditablePID interface
    }

    public void initialize(EditablePID subsystem, double p, double i, double d) {
        if (this.initialized)
            throw new IllegalStateException("PIDEditor for subsystem " + SendableRegistry.getName(subsystem) + " was already initialized.");

        // when called by a subsystem, pass <code>this</code> as the first implicit parameter
        this.subsystem = subsystem;

        this.pKey = SendableRegistry.getName(subsystem) + " P";
        this.iKey = SendableRegistry.getName(subsystem) + " I";
        this.dKey = SendableRegistry.getName(subsystem) + " D";

        this.initialized = true;

        // unless if we can have subsystems inherit PID values from EditablePID,
        // initial values will need to be passed as parameters
        SmartDashboard.putNumber(pKey, p);
        SmartDashboard.putNumber(iKey, i);
        SmartDashboard.putNumber(dKey, d);
    }

    public EditablePID getSubsystem() {
        return subsystem;
    }

    public double getP(double defaultP) {
        return SmartDashboard.getNumber(pKey, defaultP);
    }

    public double getI(double defaultI) {
        return SmartDashboard.getNumber(iKey, defaultI);
    }

    public double getD(double defaultD) {
        return SmartDashboard.getNumber(dKey, defaultD);
    }
}