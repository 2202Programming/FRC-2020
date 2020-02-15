package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousClimbCommand extends CommandBase
{
    private ClimberSubsystem climber;

    private double rotStartTime = 0;
    private double wnStartTime = 0;

    private final double ROT_DUR_TIME = 2;
    private final double WN_DUR_TIME = 2;

    private RotState rotState = RotState.FOLDED;
    private WinchState wnState = WinchState.RETRACTED;
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

    /**
     * Checks the motor time and if the time is reached it stops them
     */
    private void checkMotorStop()
    {
        if (rotState == RotState.UNFOLDING && ROT_DUR_TIME <= (System.currentTimeMillis() - rotStartTime))
        {
            rotState = RotState.UNFOLDED;
            climber.setRotSpeed(0);
        }

        else if (rotState == RotState.FOLDING && ROT_DUR_TIME <= (System.currentTimeMillis() - rotStartTime))
        {
            rotState = RotState.FOLDED;
            climber.setRotSpeed(0);
        }

        if (wnState == WinchState.EXTENDING && WN_DUR_TIME <= (System.currentTimeMillis() - wnStartTime))
        {
            wnState = WinchState.EXTENDED;
            climber.setWinchSpeed(0);

        }

        else if (wnState == WinchState.RETRACTING && WN_DUR_TIME <= (System.currentTimeMillis() - wnStartTime))
        {
            wnState = WinchState.RETRACTED;
            climber.setWinchSpeed(0);
        }
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