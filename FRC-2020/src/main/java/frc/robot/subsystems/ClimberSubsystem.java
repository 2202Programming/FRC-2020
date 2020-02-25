package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * Subsystem that manages the climbing arm motors.
 */
public class ClimberSubsystem extends SubsystemBase
{
    //region
    private DoubleSolenoid armSolenoid = new DoubleSolenoid(6, 7); //Arm valves TODO:put in constants
    //endregion

    //region Motors
    private TalonSRX eRotServo = new TalonSRX(18); // Arm rotation motor TODO:put in constants
    private ControlMode mode = ControlMode.Position;
    private int countsPerRotation = 4096;
    
    private CANSparkMax wnSparkMax = new CANSparkMax(WN_SPARKMAX_CANID, MotorType.kBrushless); //Winch motor - extend / retract arm
    //endregion

    public ClimberSubsystem() 
    {
        armSolenoid.set(DoubleSolenoid.Value.kOff);
        configureTalon(false, kp, ki, kd, kf);

        eRotServo.set(0);
        wnSparkMax.set(0);
        wnSparkMax.setIdleMode(IdleMode.kBrake);
    }

    void configureTalon(double kp, double ki, double kd, double kf){
		eRotServo.i(port,reverse);
		mode = ControlMode.Position;
		part.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		part.config_kP(0, kp, 0);
		part.config_kI(0, ki, 0);
		part.config_kD(0, kd, 0);
		part.config_kF(0, kf, 0);
		countsPerRotation = 4096;
		part.setIntegralAccumulator(0.0,0,0);
		part.setSelectedSensorPosition(0, 0, 0);
	}


    /**
     * Takes in a number and clamps it so it's min is -1 and it's max is 1
     * @param in Input number
     * @return Clamped value
     */
    private double validateDouble(double in)
    {
        if (in < -1)
        {
            return -1;
        }

        else if (in > 1)
        {
            return 1;
        }

        else
        {
            return in;
        }
    }

    /**
     * Set the position of the arm rotation servo
     * @param speed The position to unfold the arm to (between 0 - full left, and 1 - full right)
     */
    public void setRotPos(double pos)
    {
        eRotServo.setPosition(pos); //Set servo position
    }

    /**
     * Set the speed of the winch motor.
     * 1 reels the winch in.
     * -1 loosens the winch.
     * @param speed The speed at which to extend the arm (-1 to 1)
     */
    public void setWinchSpeed(double speed)
    {
        speed = validateDouble(speed);
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
        SmartDashboard.putNumber("Arm rotation position", eRotServo.get());
        SmartDashboard.putNumber("Winch speed", wnSparkMax.get());
    }
}