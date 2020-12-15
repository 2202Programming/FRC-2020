package frc.robot.util.misc;

import static frc.robot.Constants.DT;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * PIDFController - extends current (2020) pidcontroller to include a feed
 * forward gain which is not currently part of the WPILib version.
 * 
 * This is useful for holding values for devices like the talon SRX or sparkMax
 * which may have a feed forward gain.
 * 
 * 
 */
public class PIDFController extends PIDController {
    double m_Kf = 0.0;

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kf, DT);
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period) {
        super(Kp, Ki, Kd, period);
    }

    // Accessors for the Kf
    public double getF() {
        return m_Kf;
    }

    public void setF(double Kf) {
        m_Kf = Kf;
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     */
    @Override
    public double calculate(double measurement, double setpoint) {
        // Set setpoint to provided value
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     */
    @Override
    public double calculate(double measurement) {
        return calculate(measurement) + (m_Kf * getSetpoint());
    }

    /**
     * Copied from base class and feed forward added.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDFController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty("f", this::getF, this::setF);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }

}
