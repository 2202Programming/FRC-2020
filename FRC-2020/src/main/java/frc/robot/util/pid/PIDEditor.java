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
public class PIDEditor {
    /**
     * The subsystem using this object.
     */
    private EditablePID subsystem;
    /**
     * The respective key for the subsystem, to use for the SmartDashboard.
     */
    private String pKey, iKey, dKey;
    /**
     * Whether or not the PIDEditor was already initialized. The PIDEditor should only be initialized once.
     */
    private boolean initialized = false;

    /**
     * Constructs a PIDEditor for the subsystem. Each subsystem with PID values that are intended to be
     * editable and updatable dynamically should implement the EditablePID interface to construct and use
     * one. The constructor is empty to allow the interface to construct one initially.
     */
    public PIDEditor() {
    }

    /**
     * Initializes the PIDEditor for the subsystem implementing the EditablePID interface. Can only be
     * called once, and should be called once before any other methods on it are invoked.
     * @param subsystem the subsystem with editable PID values. Should be set to <code>this</code>
     */
    public void initialize(EditablePID subsystem) {
        if (this.initialized)
            throw new IllegalStateException("PIDEditor for subsystem " + SendableRegistry.getName(subsystem) + " was already initialized.");

        this.subsystem = subsystem;

        this.pKey = SendableRegistry.getName(subsystem) + " P";
        this.iKey = SendableRegistry.getName(subsystem) + " I";
        this.dKey = SendableRegistry.getName(subsystem) + " D";

        this.initialized = true;

        SmartDashboard.putNumber(pKey, subsystem.getP());
        SmartDashboard.putNumber(iKey, subsystem.getI());
        SmartDashboard.putNumber(dKey, subsystem.getD());
    }

    /**
     * Gets the subsystem with which the PIDEditor is associated.
     * @return the respective subsystem
     */
    public EditablePID getSubsystem() {
        return subsystem;
    }

    /**
     * Gets the key used in the SmartDashboard to dynamically update the P value for this subsystem.
     * @return the pKey
     */
    public String getPKey() {
        return pKey;
    }

    /**
     * Gets the key used in the SmartDashboard to dynamically update the I value for this subsystem.
     * @return the iKey
     */
    public String getIKey() {
        return iKey;
    }

    /**
     * Gets the key used in the SmartDashboard to dynamically update the D value for this subsystem.
     * @return the dKey
     */
    public String getDKey() {
        return dKey;
    }

    /**
     * Gets the new P value.
     * @return the new p
     */
    public double updateP() {
        double newP = SmartDashboard.getNumber(pKey, subsystem.getP());
        return newP;
    }

    /**
     * Gets the new I value.
     * @return the new i
     */
    public double updateI() {
        double newI = SmartDashboard.getNumber(iKey, subsystem.getI());
        return newI;
    }

    /**
     * Gets the new D value.
     * @return the new d
     */
    public double updateD() {
        double newD = SmartDashboard.getNumber(dKey, subsystem.getD());
        return newD;
    }
}