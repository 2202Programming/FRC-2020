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

    private ArmState state = ArmState.FOLDED;

    /** Initial knowledge of how the subsystem is to be run
     * 775 motor used for rotation(Talon controller)
     * neo motor used to shoot arm up(sparkmax controller)
     * mechanical stop for rotation
     * Ask Kevin for more info from Kevin
     */
    public ClimberSubsystem() 
    {
        //Set the motors to brake for testing purposes, feel free to change if needed
        eSparkMax.setIdleMode(IdleMode.kBrake);
        eROTSparkMax.setIdleMode(IdleMode.kBrake);
        wnSparkMax.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() 
    {
        
    }

    /**
     * Fold the robot's arm
     */
    public void foldArm()
    {
        
    }

    /**
     * Unfold's the arm from the robot
     */
    public void unfoldArm()
    {

    }

    /**
     * Extend upper arm to begin grabbing
     */
    public void extendArm()
    {
        
    }

    public enum ArmState
    {
        FOLDED,
        UNFOLDED,
        EXTENDED,
        GRABBED
    }
}