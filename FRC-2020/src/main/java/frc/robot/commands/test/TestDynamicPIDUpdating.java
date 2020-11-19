package frc.robot.commands.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Test command for dynamically updating PID values via SmartDashboard.
 * TODO: Test this program by constructing this in the program, simulating it, and manually altering a value in the SmartDashboard. The program should log the changes and, if in debug mode, reach a breakpoint.
 * @author Kevin Li
 */
public class TestDynamicPIDUpdating extends CommandBase {
    public static final double DEFAULT_P = 1.0, DEFAULT_I = 1.0, DEFAULT_D = 1.0; // TODO: change as necessary

    private static double Kp = DEFAULT_P, Ki = DEFAULT_I, Kd = DEFAULT_D;

    private double initialP, initialI, initialD;

    /**
     * Creates a test command for dynamically updating PID values.
     * Uses the current (initially default) PID values.
     */
    public TestDynamicPIDUpdating() {
        // uses current PID values

        initializeOriginalPID();

        displayPIDValues();
    }

    /**
     * Creates a test command for dynamically updating PID values.
     * @param Kp the initial P value
     * @param Ki the initial I value
     * @param Kd the initial D value
     */
    public TestDynamicPIDUpdating(double Kp, double Ki, double Kd) {
        TestDynamicPIDUpdating.Kp = Kp;
        TestDynamicPIDUpdating.Ki = Ki;
        TestDynamicPIDUpdating.Kd = Kd;

        initializeOriginalPID();

        displayPIDValues();
    }

    /**
     * Helper method that initalizes the initial PID values.
     */
    private void initializeOriginalPID() {
        initialP = Kp;
        initialI = Ki;
        initialD = Kd;
    }

    /**
     * Displays the PID values on the SmartDashboard.
     */
    private void displayPIDValues() {
        SmartDashboard.putNumber("Test P", Kp);
        SmartDashboard.putNumber("Test I", Ki);
        SmartDashboard.putNumber("Test D", Kd);
    }

    @Override
    public void execute() {
        setP(SmartDashboard.getNumber("Test P", Kp));
        setI(SmartDashboard.getNumber("Test I", Ki));
        setD(SmartDashboard.getNumber("Test D", Kd));
    }

    @Override
    public boolean isFinished() {
        // finishes when a value is changed
        boolean isFinished = (Kp != initialP) || (Ki != initialI) || (Kd != initialD);
        if (isFinished) {
            log();
            return true;
        }
        return false;
    }

    /**
     * Logs the initial and final PID values.
     */
    private void log() {
        System.out.println("Initial P: " + initialP);
        System.out.println("Final P: " + Kp);
        System.out.println("Initial I: " + initialI);
        System.out.println("Final I: " + Ki);
        System.out.println("Initial D: " + initialD);
        System.out.println("Final D: " + Kd);
    }

    /**
     * Changes the P-value.
     * @param newP the new P-value
     */
    private void setP(double newP) {
        Kp = newP;
    }

    /**
     * Changes the I-value.
     * @param newI the new I-value
     */
    private void setI(double newI) {
        Ki = newI;
    }

    /**
     * Changes the D-value.
     * @param newD the new D-value
     */
    private void setD(double newD) {
        Kd = newD;
    }
}