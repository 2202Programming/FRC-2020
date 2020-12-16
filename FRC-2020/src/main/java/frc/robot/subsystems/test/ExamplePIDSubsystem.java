package frc.robot.subsystems.test;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExamplePIDSubsystem extends SubsystemBase {
    public static final String KEY = "Example PID Subsystem";

    private PIDController pidCtrlLow, pidCtrlHigh;
    
    public ExamplePIDSubsystem(double lowP, double lowI, double lowD, double highP, double highI, double highD) {
        pidCtrlLow = new PIDController(lowP, lowI, lowD);
        pidCtrlHigh = new PIDController(highP, highI, highD);

        SmartDashboard.putData(KEY + " Low", pidCtrlHigh);
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
}