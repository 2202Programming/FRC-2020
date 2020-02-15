package frc.robot.subsystems;

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
    //region Motors
    //private CANSparkMax eSparkMax = new CANSparkMax(Constants.E_SPARKMAX_CANID, MotorType.kBrushed);  //Changed (invalid?)
    private ServoMotor eRotSparkMax = new CANSparkMax(Constants.E_ROT_SPARKMAX_PWM, MotorType.kBrushless); //Arm rotation motor (TODO: class)
    private CANSparkMax wnSparkMax = new CANSparkMax(Constants.WN_SPARKMAX_CANID, MotorType.kBrushless); //Winch motor - extend / retract arm
    //endregion

    /* Initial knowledge of how the subsystem is to be run
     * 775 motor used for rotation(Talon controller)
     * neo motor used to shoot arm up(sparkmax controller)
     * mechanical stop for rotation
     * Ask Kevin for more info from Kevin
     */

    /**
     * 
     */
    public ClimberSubsystem() 
    {
        //eSparkMax.setIdleMode(IdleMode.kBrake);
        eRotSparkMax.setIdleMode(IdleMode.kBrake);
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
     * Set the speed of the arm rotations motor
     * @param speed The speed at which to unfold the arm (-1 to 1)
     */
    public void setRotSpeed(double speed)
    {
        speed = validateDouble(speed);
        eRotSparkMax.set(speed); //Set motor speed
    }

    /**
     * Set the speed of the winch motor
     * @param speed The speed at which to extend the arm (-1 to 1)
     */
    public void setWinchSpeed(double speed)
    {
        speed = validateDouble(speed);
        wnSparkMax.set(speed); //Set motor speed
    }

    public void log()
    {
        SmartDashboard.putNumber("Arm rotation speed", eRotSparkMax.get());
        SmartDashboard.putNumber("Winch speed", wnSparkMax.get());
    }
}