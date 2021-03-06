package frc.robot.util.misc;

import static frc.robot.Constants.DT;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * PIDFController - extends current (2020) pidcontroller to include a feed
 * forward gain which is not currently part of the WPILib version.
 * 
 * This is useful for holding values for devices like the talon SRX or sparkMax
 * which may have a feed forward gain or Izone.
 * 
 * 2/16/21 added CopyTo helper functions
 * 
 */
public class PIDFController extends PIDController {
    double m_Kf = 0.0;
    double m_izone = 0.0;

    public PIDFController(double Kp, double Ki, double Kd, double Kf) {
        this(Kp, Ki, Kd, Kf, DT);
    }

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double period) {
        super(Kp, Ki, Kd, period);
        setF(Kf);
    }

    // Accessors for the Kf
    public double getF() {
        return m_Kf;
    }

    public void setF(double Kf) {
        m_Kf = Kf;
    }

   public void setIzone(double m_izone) {
      this.m_izone = m_izone;
    }

    public double getIzone() {
      return m_izone; 
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
        /*builder.setSmartDashboardType("PIDFController");
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("F", this::getF, this::setF);
        builder.addDoubleProperty("Iz", this::getIzone, this::setIzone);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);*/
    }

    /**
     * 
     * copyTo()  copies this pid's values down to a hardward PID implementation
     * @param dest  device 
     * @param slot  control slot on device
     */

    public void copyTo(CANPIDController dest, int slot) {
      dest.setP(this.getP(), slot);
      dest.setI(this.getI(), slot);
      dest.setD(this.getD(), slot);
      dest.setFF(this.getF(), slot);
      dest.setIZone(this.getIzone(), slot);
    }

    public void copyTo(SlotConfiguration dest) {
      dest.kP = this.getP();
      dest.kI = this.getI();
      dest.kD = this.getD();
      dest.kF = this.getF();
      dest.integralZone = this.getIzone();
    }

}
