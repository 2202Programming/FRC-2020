package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Subsystem that manages the climbing arm motors.
 */
public class ClimberSubsystem extends SubsystemBase
{
    //region
    private DoubleSolenoid armSolenoid = new DoubleSolenoid(6, 7); //Arm valves
    //endregion

    //region Motors
    private Servo eRotServo = new Servo(Constants.E_ROT_SERVO_PWM); //Arm rotation motor
    private CANSparkMax wnSparkMax = new CANSparkMax(Constants.WN_SPARKMAX_CANID, MotorType.kBrushless); //Winch motor - extend / retract arm
    //endregion

    public ClimberSubsystem() 
    {
        armSolenoid.set(DoubleSolenoid.Value.kOff);
        eRotServo.set(0);
        wnSparkMax.set(0);
        wnSparkMax.setIdleMode(IdleMode.kBrake);
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