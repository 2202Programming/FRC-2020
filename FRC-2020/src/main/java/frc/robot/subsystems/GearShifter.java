package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.*;

public class GearShifter implements Subsystem {
    // define the gear ratios for high and low gear
    private final double K_shaft = (12.0 /30.0) * (24.0 / 60.0); //motor and output stage
    //shifter stage with 
    public final double K_high = 1.0 * K_shaft;
    public final double K_low = (1.0 / 2.65) * K_shaft; 
    private boolean autoShiftEnabled = false;
    
    public enum Gear {
        HIGH_GEAR(DoubleSolenoid.Value.kForward), 
        LOW_GEAR(DoubleSolenoid.Value.kReverse);

        private final DoubleSolenoid.Value gearCode;    

        Gear(DoubleSolenoid.Value value) {
            gearCode = value;
        }

        public DoubleSolenoid.Value solenoidCmd() {
            return this.gearCode;
        }
    }

    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(GEARSHIFT_PCM_ID, GEARSHIFTUP_SOLENOID_PCM,
            GEARSHIFTDOWN_SOLENOID_PCM);

    // State
    private Gear curGear = Gear.LOW_GEAR;

    public GearShifter() {

    }
    
    public Gear getCurGear() {
        return curGear;
    }

    public void shiftUp() {
        gearShiftSolenoid.set(Gear.HIGH_GEAR.solenoidCmd());
        curGear = Gear.HIGH_GEAR;
    }

    public void shiftDown() {
        gearShiftSolenoid.set(Gear.LOW_GEAR.solenoidCmd());
        curGear = Gear.LOW_GEAR;
    }

    public void setAutoShift(boolean autoShifting) {
        autoShiftEnabled = autoShifting;
    }

    public boolean isAutoShift() {
        return autoShiftEnabled;
    }

    public void log() {
        SmartDashboard.putBoolean("Auto-Shift Enabled", autoShiftEnabled);
        SmartDashboard.putString("Current Gear", getCurGear().toString());
    }

    //current gear ratio
    public double getGearRatio() { 
        return (Gear.HIGH_GEAR == curGear) ? K_high : K_low;
    }

    // specific gear ratio - informational only
    public double getGearRatio(Gear g) {
        return (Gear.HIGH_GEAR == g) ? K_high : K_low;
    }

}