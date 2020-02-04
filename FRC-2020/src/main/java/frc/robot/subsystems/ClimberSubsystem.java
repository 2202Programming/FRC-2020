package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Subsystem that manages the climbing arm.
 * Manages lifting of robot and the folding of the arm.
 */
public class ClimberSubsystem extends SubsystemBase
{
    //region Motors
    private CANSparkMax eSparkMax = new CANSparkMax(Constants.E_SPARKMAX_CANID, MotorType.kBrushed);  //Extension motor - rotation
    private CANSparkMax eROTSparkMax = new CANSparkMax(Constants.E_ROT_SPARKMAX_CANID, MotorType.kBrushless); //Extension arm rotation motor - maybe unused?
    private CANSparkMax wnSparkMax = new CANSparkMax(Constants.WN_SPARKMAX_CANID, MotorType.kBrushless); //Winch motor - extend / retract arm
    //endregion

    private RotState rotState = RotState.FOLDED;
    private WinchState wnState = WinchState.RETRACTED;
    
    public RotState getRotState()
    {
        return rotState;
    }

    public WinchState getWinchState()
    {
        return wnState;
    }

    private long rotStartTime = 0;
    private long wnStartTime = 0;

    //TODO: calibrate these times so that they spin the motors for the correct amount of time
    public final double ROT_DUR_TIME;
    public final double WN_DUR_TIME;

    /**
     * Rotation speed. Has to be between -1.0 and 1.0
     */
    public static final double ROT_SPEED = .25;

    /**
     * Rotation speed. Has to be between -1.0 and 1.0
     */
    public static final double WN_SPEED = .25;
    
    /** Initial knowledge of how the subsystem is to be run
     * 775 motor used for rotation(Talon controller)
     * neo motor used to shoot arm up(sparkmax controller)
     * mechanical stop for rotation
     * Ask Kevin for more info from Kevin
     */

    /**
     * 
     * @param rotDuration How long we turn the rotation motor
     * @param winchDuration How long we turn the winch motor
     */
    public ClimberSubsystem(double rotDuration, double winchDuration) 
    {
        ROT_DUR_TIME = rotDuration;
        WN_DUR_TIME = winchDuration;

        //Set the motors to brake for testing purposes, feel free to change if needed
        eSparkMax.setIdleMode(IdleMode.kBrake);
        eROTSparkMax.setIdleMode(IdleMode.kBrake);
        wnSparkMax.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() 
    {
        if (rotState == RotState.UNFOLDING && ROT_DUR_TIME <= (System.currentTimeMillis() - rotStartTime))
        {
            rotState = RotState.UNFOLDED;
            eSparkMax.set(0);
        }

        else if (rotState == RotState.FODLING && ROT_DUR_TIME <= (System.currentTimeMillis() - rotStartTime))
        {
            rotState = RotState.FOLDED;
            eSparkMax.set(0);
        }

        if (wnState == WinchState.EXTENDING && WN_DUR_TIME <= (System.currentTimeMillis() - wnStartTime))
        {
            wnState = WinchState.EXTENDED;
            wnSparkMax.set(0);

        }

        else if (wnState == WinchState.RETRACTING && WN_DUR_TIME <= (System.currentTimeMillis() - wnStartTime))
        {
            wnState = WinchState.RETRACTED;
            wnSparkMax.set(0);
        }
    }

    /**
     * Unfold's the arm from the robot
     */
    public void unfoldArm()
    {
        //If we're unfolded/unfolding then return
        if (rotState == RotState.UNFOLDED || rotState == RotState.UNFOLDING)
        {
            return;
        }

        rotState = RotState.UNFOLDING; //Update current rotation state
        rotStartTime = System.currentTimeMillis(); //Set the timer
        eSparkMax.set(ROT_SPEED); //Set motor speed
    }

    /**
     * Fold the robot's arm
     */
    public void foldArm()
    {
        //If we're folded/folding then return
        if (rotState == RotState.FOLDED || rotState == RotState.FODLING)
        {
            return;
        }

        rotState = RotState.FODLING; //Update current rotation state
        rotStartTime = System.currentTimeMillis(); //Set the timer
        eSparkMax.set(-ROT_SPEED); //Set motor speed
    }

    /**
     * Extend upper arm
     */
    public void extendArm()
    {
        //If we're extended/extending then return
        if (wnState == WinchState.EXTENDED || wnState == WinchState.EXTENDING)
        {
            return;
        }

        wnState = WinchState.EXTENDING; //Update current winch state
        wnStartTime = System.currentTimeMillis(); //Set the timer
        wnSparkMax.set(WN_SPEED); //Set motor speed
    }

    /**
     * Retract upper arm
     */
    public void retractArm()
    {
        //If we're retracted/retracting then return
        if (wnState == WinchState.RETRACTED || wnState == WinchState.RETRACTING)
        {
            return;
        }

        wnState = WinchState.RETRACTING; //Update current winch state
        wnStartTime = System.currentTimeMillis(); //Set the timer
        wnSparkMax.set(-WN_SPEED); //Set motor speed
    }

    public enum RotState
    {
        UNFOLDED,
        UNFOLDING,
        FOLDED,
        FODLING
    }

    public enum WinchState
    {
        EXTENDED,
        EXTENDING,
        RETRACTED,
        RETRACTING
    }
}