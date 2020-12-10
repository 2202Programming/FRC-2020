package frc.robot.util.pid;

import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.EditablePID;
import frc.robot.subsystems.Log_Subsystem;

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

    public void initialize(EditablePID subsystem) {
        if (this.initialized)
            throw new IllegalStateException("PIDEditor for subsystem " + SendableRegistry.getName(subsystem) + " was already initialized.");

        // when called by a subsystem, pass <code>this</code> as the first implicit parameter
        this.subsystem = subsystem;

        this.pKey = SendableRegistry.getName(subsystem) + " P";
        this.iKey = SendableRegistry.getName(subsystem) + " I";
        this.dKey = SendableRegistry.getName(subsystem) + " D";

        this.initialized = true;

        SmartDashboard.putNumber(pKey, subsystem.getP());
        SmartDashboard.putNumber(iKey, subsystem.getI());
        SmartDashboard.putNumber(dKey, subsystem.getD());
    }

    public EditablePID getSubsystem() {
        return subsystem;
    }

    public double updateP() {
        double newP = SmartDashboard.getNumber(pKey, subsystem.getP());
        return newP;
    }

    public double updateI() {
        double newI = SmartDashboard.getNumber(iKey, subsystem.getI());
        return newI;
    }

    public double updateD() {
        double newD = SmartDashboard.getNumber(dKey, subsystem.getD());
        return newD;
    }
}