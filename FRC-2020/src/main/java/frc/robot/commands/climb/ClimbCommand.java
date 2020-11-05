package frc.robot.commands.climb;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCommand extends CommandBase
{
    private ClimberSubsystem climber;

    public ClimbCommand(ClimberSubsystem _climber)
    {
        climber = _climber;
        addRequirements(_climber);
    }

    /**
     * Called on first schedule, when the command is first started
     */
    @Override
    public void initialize() 
    {
        climber.setWinchSpeed(0);
    }

    @Override
    public void execute()
    {
        //Functionality
    }

    /**
     * Test for finish conditions to end command
     */
    @Override
    public boolean isFinished() 
    {
        return false;
    }

    /**
     * Cleanup, final command etc goes here
     */
    @Override
    public void end(boolean interrupted)
    {
        //climber.setRotSpeed(0);
        climber.setWinchSpeed(0);
    }
}