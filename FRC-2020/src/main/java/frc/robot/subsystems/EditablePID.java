package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.pid.PIDEditor;

/**
 * An interface for subsystems that use PID values that are intended to be dynamically changeable
 * via the user interface: the SmartDashboard.
 * 
 * @author Kevin Li
 */
public interface EditablePID extends Subsystem, Sendable {
    /**
     * The respective PIDEditor for this subsystem.
     */
    PIDEditor pidEditor = new PIDEditor();

    /**
     * Initializes the subsystem's PIDEditor. Should be called once and only once, at the initialization
     * or construction of the subsystem.
     */
    default void initializePIDEditor() {
        pidEditor.initialize(this); // only to be run once per subsystem (should it be changed for different modes (auto, teleop, etc.)?)
    }

    /**
     * Gets the current P value of the subsystem.
     * @return the current p
     */
    double getP();

    /**
     * Gets the current I value of the subsystem.
     * @return the current i
     */
    double getI();

    /**
     * Gets the current D value of the subsystem.
     * @return the current d
     */
    double getD();

    /**
     * Gets the updated P value for the subsystem. The implementing subsystem class should periodically
     * invoke this and set the P instance to it.
     * @return the new p
     */
    default double getNewP() {
        return pidEditor.updateP();
    }

    /**
     * Gets the updated I value for the subsystem. The implementing subsystem class should periodically
     * invoke this and set the I instance to it.
     * @return the new i
     */
    default double getNewI() {
        return pidEditor.updateI();
    }

    /**
     * Gets the updated D value for the subsystem. The implementing subsystem class should periodically
     * invoke this and set the D instance to it.
     * @return the new d
     */
    default double getNewD() {
        return pidEditor.updateD();
    }
}