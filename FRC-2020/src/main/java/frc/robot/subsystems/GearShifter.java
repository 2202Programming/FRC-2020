package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.Shifter;

import static frc.robot.Constants.*;

public class GearShifter extends SubsystemBase implements Shifter {
    // define the gear ratios for high and low gear
    private final double K_shaft = 1.0/18.75; //motor revs to wheel revs
    //shifter stage with 
    public final double K_low = 1.0 * K_shaft;     //motor rev to wheel revs
    public final double K_high = 2.65 * K_shaft;   //motor rev to wheel revs 
    private boolean autoShiftEnabled = false;
    
    public enum Gear {
        LOW_GEAR(DoubleSolenoid.Value.kForward),    // dpl - fliped assignment to match comp bot 2/24/20
        HIGH_GEAR(DoubleSolenoid.Value.kReverse);

        private final DoubleSolenoid.Value gearCode;    

        Gear(DoubleSolenoid.Value value) {
            gearCode = value;
        }

        public DoubleSolenoid.Value solenoidCmd() {
            return this.gearCode;
        }
    }

    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(GEARSHIFT_PCM_CAN_ID, GEARSHIFTUP_SOLENOID_PCM,
            GEARSHIFTDOWN_SOLENOID_PCM);

    // State
    private Gear curGear = Gear.LOW_GEAR;

    public GearShifter() {
    }
    
    @Override
    public Gear getCurrentGear() {
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

    

    public void log() {
        SmartDashboard.putBoolean("Auto-Shift Enabled", autoShiftEnabled);
        SmartDashboard.putString("Current Gear", getCurrentGear().toString());
    }

    //current gear ratio
    public double getGearRatio() { 
        return (Gear.HIGH_GEAR == curGear) ? K_high : K_low;
    }

    // specific gear ratio - informational only
    public double getGearRatio(Gear g) {
        return (Gear.HIGH_GEAR == g) ? K_high : K_low;
    }

    /*
    *  AutoSifter interface 
    */

    @Override
    public boolean isAutoShiftEnabled() {
        return autoShiftEnabled;
    }

    @Override
    public boolean enableAutoShift() {
        autoShiftEnabled = true;
        return autoShiftEnabled;
    }

    @Override
    public boolean disableAutoShift() {
       autoShiftEnabled = false;
       return autoShiftEnabled;
    }
}