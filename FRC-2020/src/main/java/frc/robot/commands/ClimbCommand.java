package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbCommand extends CommandBase
{
    private ClimberSubsystem cSubsystem;

    public ClimbCommand(ClimberSubsystem sub)
    {
        cSubsystem = sub;
        addRequirements(sub);
    } 

    public ClimbCommand() 
    {
        ClimberSubsystem sub = new ClimberSubsystem(2, 2);
        cSubsystem = sub;
        addRequirements(sub);
    }

    @Override
    public void execute()
    {
        //TODO: setup arm and winch behaviour
    }
}