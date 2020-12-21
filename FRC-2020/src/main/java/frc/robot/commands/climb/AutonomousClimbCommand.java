package frc.robot.commands.climb;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousClimbCommand extends CommandBase
{
    private ClimberSubsystem climber;
    @SuppressWarnings("unused")
    private double rotStartTime = 0;
    @SuppressWarnings("unused")
    private double wnStartTime = 0;

    @SuppressWarnings("unused")
    private final double ROT_DUR_TIME = 2;
    @SuppressWarnings("unused")
    private final double WN_DUR_TIME = 2;

    private RotState rotState = RotState.FOLDED;
    private WinchState wnState = WinchState.RETRACTED;
    @SuppressWarnings("unused")
    private TargetState targetState;

    public RotState getRotState()
    {
        return rotState;
    }

    public WinchState getWinchState()
    {
        return wnState;
    }

    public AutonomousClimbCommand(ClimberSubsystem _climber, TargetState targState)
    {
        targetState = targState;
        climber = _climber;
        addRequirements(_climber);
    }

    @Override
    public void initialize() 
    {
        climber.setWinchSpeed(0);


    }

    public void fold()
    {

    }

    public void climb()
    {
        
    }

    public enum TargetState
    {
        FOLD,
        CLIMB
    }

    public enum RotState
    {
        UNFOLDED,
        UNFOLDING,
        FOLDED,
        FOLDING
    }

    public enum WinchState
    {
        EXTENDED,
        EXTENDING,
        RETRACTED,
        RETRACTING
    }
}