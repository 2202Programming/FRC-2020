package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SetDefaultCmd extends InstantCommand{
    private Subsystem system;
    private Command defaultCommand;
    /**
     * Command to change a default command of a subsystem.
     */
    public SetDefaultCmd(Subsystem system, Command newDefault){
        addRequirements(system);
        this.system = system;
        defaultCommand = newDefault;
    }

    @Override
    public void initialize() {
        Command oldDefault = system.getDefaultCommand();
        system.setDefaultCommand(defaultCommand);
        oldDefault.cancel();
        System.out.println("Set Default Command to: " + defaultCommand.getName());
    }
}