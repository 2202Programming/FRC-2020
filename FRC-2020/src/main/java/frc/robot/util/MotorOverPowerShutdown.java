package frc.robot.input.triggers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MotorOverPowerShutdown extends Trigger implements PIDSource {
    final int TAP_MAX = 25;   // this is 0.5 seconds normally
    WPI_TalonSRX motor;
    double powerLimit;
    double avgPower;          //watts
    Command saveMotorCmd;
    LinearDigitalFilter movingWindow;

    public MotorOverPowerShutdown(WPI_TalonSRX motor, double powerLimit, double seconds) {
        this.powerLimit = powerLimit;
        this.motor = motor;    
        this.avgPower = 0.0;
        int taps = (int) Math.floor(seconds / Robot.dT);
    
        // build a moving average window
        movingWindow = LinearDigitalFilter.movingAverage(this, taps);
        this.saveMotorCmd = new SaveMotor();

        //install the command and hope it is never used
        this.whenActive(this.saveMotorCmd);

        System.out.println("OverPower " +motor.getName() + " watts= " + powerLimit + " - for testing only");
    }

    @Override
    public boolean get() {
        // this is called each frame, so call pidGet() here.
        // Not really a pid, but this is how the filter class works
        pidGet();   //reads values, computes power and saves in the window
        // look for too much average power 
        if (movingWindow.get() > powerLimit) return true;
        return false;
    }

    // monitor power
    double readPower() {
        double oi = motor.getOutputCurrent();
        double ov = motor.getMotorOutputVoltage();
        return Math.abs(oi*ov);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        //dpl - don't think this matters
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return  PIDSourceType.kDisplacement;    //dpl should not matter for our use as a filter
    }

    // inserts value into the filter, called by filter.
    @Override
    public double pidGet() {
        return readPower();  //value used for filter
	}

    // SaveMotor will disable the motor and set speed to zero of the overpower triggers
    class SaveMotor extends Command {
        SaveMotor() {           
        }

        @Override
        public void execute() {
            motor.set(0.0);
            motor.disable();

            //Make noise
            System.out.println("****MOTOR POWER TRIGGERED**** -->" + motor.getName() );
        }
        
        // keep in the safe state, this command will have to get kicked out.
        @Override
        public boolean isFinished() { 
            return false; 
        }
    }

}