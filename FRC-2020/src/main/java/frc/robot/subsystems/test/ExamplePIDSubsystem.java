package frc.robot.subsystems.test;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.Logger;
// import frc.robot.triggers.PIDUpdateButtonTrigger;

public class ExamplePIDSubsystem extends SubsystemBase implements Logger {
    public static final String KEY = "Example PID Subsystem";

    private final PIDController pidCtrlLow, pidCtrlHigh;
    //private final PIDUpdateButtonTrigger pidButtonLow, pidButtonHigh;
    
    public ExamplePIDSubsystem(double lowP, double lowI, double lowD, double highP, double highI, double highD) {
        pidCtrlLow = new PIDController(lowP, lowI, lowD);
        pidCtrlHigh = new PIDController(highP, highI, highD);
        //pidButtonLow = new PIDUpdateButtonTrigger(pidCtrlLow);
        //pidButtonHigh = new PIDUpdateButtonTrigger(pidCtrlHigh);

        // Note: data can be dynamically updated in Shuffleboard - is a button necessary?
        SmartDashboard.putData(KEY + " Low", pidCtrlLow);
        SmartDashboard.putData(KEY + " High", pidCtrlHigh);
    }

    public double getLowP() {
        return pidCtrlLow.getP();
    }

    public double getLowI() {
        return pidCtrlLow.getI();
    }

    public double getLowD() {
        return pidCtrlLow.getD();
    }

    public double getHighP() {
        return pidCtrlHigh.getP();
    }

    public double getHighI() {
        return pidCtrlHigh.getI();
    }

    public double getHighD() {
        return pidCtrlHigh.getD();
    }

    @Override
    public void log() {
        // TODO: for testing purposes
        System.out.println("Low PID: " + getLowP() + ", " + getLowI() + ", " + getLowD());
        System.out.println("High PID: " + getHighP() + ", " + getHighI() + ", " + getHighD());
    }
}