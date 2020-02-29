package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.util.misc.Gains;
import frc.robot.util.misc.MathUtil;

import static frc.robot.Constants.*;

/**
 * Subsystem that manages the climbing arm motors.
 */
public class ClimberSubsystem extends SubsystemBase
{
    private DoubleSolenoid armSolenoid = new DoubleSolenoid(CLIMBER_PCM_CAN_ID, 
                ARMSOLENOID_LOW_CANID, ARMSOLENOID_HIGH_CANID);
        
    private final Spark armRotationMotor = new Spark(CLIMB_ARM_TALON_CANID); // Arm rotation motor 

    /*
    //Talon configuration constants
    private final ControlMode eRotMode = ControlMode.Position;
    private final int countsPerRotation = 4096;
    private final int PID_SLOT = 0;
    private final int kTO = 0;      // comms timeout for Talons
    Gains eRotGains = new Gains(0.2, 0, 0, 0, 0, 0);
    private final int posErrorLimit = 20;  // sensor counts
    private final double kGearing = 200.0; // motor rev to arm rev todo:fix gearing
    private final double kCounts_deg = countsPerRotation * kGearing / 90.0;  // 
    */
    
    private CANSparkMax wnSparkMax = new CANSparkMax(WN_SPARKMAX_CANID, MotorType.kBrushless); //Winch motor - extend / retract arm

    public ClimberSubsystem() 
    {
        armSolenoid.set(DoubleSolenoid.Value.kOff);
        //configureTalon(armRotationMotor, PID_SLOT, eRotGains, false);

        wnSparkMax.set(0);
        wnSparkMax.setIdleMode(IdleMode.kBrake);
    }

    /*
    //Talon configuration
    void configureTalon(TalonSRX c, int slot, Gains g, boolean invert) {
        // setup the closed loop pid in the Talon
        c.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, slot, kTO);
        c.setInverted(invert);
		c.config_kP(slot, g.kP, kTO);
		c.config_kI(slot, g.kI, kTO);
		c.config_kD(slot, g.kD, kTO);
		c.config_kF(slot, g.kF, kTO);
        c.config_IntegralZone(slot, g.kIzone, kTO);
		c.configClosedLoopPeakOutput(slot, g.kPeakOutput, kTO);
		c.configAllowableClosedloopError(slot, posErrorLimit, kTO);

        resetTalonPosition(c, slot);
    }
    
    void resetTalonPosition(TalonSRX c, int slot){
		c.setIntegralAccumulator(0.0, slot, kTO);
		c.setSelectedSensorPosition(0, slot, kTO);
    }
    */
    
    //Set the speed of the arm rotation motor
    //There's a hard limit so it will stop at a specific angle
    public void setRotationSpeed(double speed) {
        armRotationMotor.setPosition(speed);
    }

    /*
    /**
     * Set the position of the arm rotation using Talon position control
     * Units should be degrees.
     * @param pos
     
    public void setRotPos(double pos) 
    {   
        double feedFwdTerm = 0.0;  // could be func of pos
        pos *= kCounts_deg;        // scale to count target
        armRotationMotor.set(eRotMode, pos, DemandType.ArbitraryFeedForward, feedFwdTerm);  //Set servo position
    }
    */

    /**
     * Set the speed of the winch motor.
     * 1 reels the winch in.
     * -1 loosens the winch.
     * @param speed The speed at which to extend the arm (-1 to 1)
     */
    public void setWinchSpeed(double speed)
    {
        speed = MathUtil.limit(speed, -1.0, 1.0);
        wnSparkMax.set(speed); //Set motor speed
    }

    public double getWinchPosition() {
        return wnSparkMax.getEncoder().getPosition();
    }

    /**
     * Extend arm
     */
    public void extendArm()
    {
        armSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Retract arm
     */
    public void retractArm()
    {
        armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Stop arm in place
     */
    public void stopArm()
    {
        armSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void log()
    {
        SmartDashboard.putNumber("Arm rotation position", armRotationMotor.getPosition());
        SmartDashboard.putNumber("Winch speed", wnSparkMax.get());
    }
}