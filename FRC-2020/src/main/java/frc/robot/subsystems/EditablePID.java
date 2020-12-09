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

    default void initializePIDEditor(int p, int i, int d) {
        pidEditor.initialize(this, p, i, d); // only to be run once per subsystem (should it be changed for different modes (auto, teleop, etc.)?)
    }

    default double getNewP(double currentP) {
        return pidEditor.getP(currentP);
    }

    default double getNewI(double currentI) {
        return pidEditor.getI(currentI);
    }

    default double getNewD(double currentD) {
        return pidEditor.getD(currentD);
    }
}