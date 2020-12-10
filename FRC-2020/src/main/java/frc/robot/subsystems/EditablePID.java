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
    PIDEditor pidEditor = new PIDEditor();

    default void initializePIDEditor() {
        pidEditor.initialize(this); // only to be run once per subsystem (should it be changed for different modes (auto, teleop, etc.)?)
    }

    double getP();

    double getI();

    double getD();

    default double getNewP() {
        return pidEditor.updateP();
    }

    default double getNewI() {
        return pidEditor.updateI();
    }

    default double getNewD() {
        return pidEditor.updateD();
    }
}