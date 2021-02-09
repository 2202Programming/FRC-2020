package frc.robot.subsystems;

import static frc.robot.Constants.GEARSHIFTDOWN_SOLENOID_PCM;
import static frc.robot.Constants.GEARSHIFTUP_SOLENOID_PCM;
import static frc.robot.Constants.GEARSHIFT_PCM_CAN_ID;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ifx.Shifter;

public class GearShifter extends SubsystemBase implements Shifter {
    // define the gear ratios for high and low gear
    private final double K_shaft = 1.0/18.75; //motor revs to wheel revs
    //shifter stage with 
    public final double K_low = 1.0 * K_shaft;     //motor rev to wheel revs
    public final double K_high = 2.65 * K_shaft;   //motor rev to wheel revs 
    private boolean autoShiftEnabled = false;
    
    enum Solnoid {
        LOW_GEAR(DoubleSolenoid.Value.kForward),    
        HIGH_GEAR(DoubleSolenoid.Value.kReverse);

        private final DoubleSolenoid.Value direction;    

        Solnoid(DoubleSolenoid.Value value) {
            direction = value;
        }
    }

    private DoubleSolenoid gearShiftSolenoid = new DoubleSolenoid(GEARSHIFT_PCM_CAN_ID, GEARSHIFTUP_SOLENOID_PCM,
            GEARSHIFTDOWN_SOLENOID_PCM);

    // State, mirror solnoid
    private Solnoid curGear = Solnoid.LOW_GEAR;

    public GearShifter() {
    }
    
    @Override
    public Gear getCurrentGear() {
       return (curGear == Solnoid.HIGH_GEAR ) ? Gear.HIGH : Gear.LOW;
    }

    public void shiftUp() {
        curGear = Solnoid.HIGH_GEAR;
        gearShiftSolenoid.set(curGear.direction); 
    }

    public void shiftDown() {
       curGear = Solnoid.LOW_GEAR;
       gearShiftSolenoid.set(curGear.direction); 
    }

    public void log() {
        SmartDashboard.putBoolean("Auto-Shift Enabled", autoShiftEnabled);
        SmartDashboard.putString("Current Gear", getCurrentGear().toString());
    }

    //current gear ratio
    public double getGearRatio() { 
        return (Solnoid.HIGH_GEAR == curGear) ? K_high : K_low;
    }

    // specific gear ratio
    public double getGearRatio(Gear g) {
        return (Gear.HIGH == g) ? K_high : K_low;
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