package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static frc.robot.Constants.*;

public class GearShifter implements Subsystem {
    

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
}