package frc.robot.commands.generic;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CancelCommand extends InstantCommand {
    private Command command;
/**
 * 
 * @param cmd  Command to cancel 
 */
    public CancelCommand(Command cmd) {
        command = cmd;
    }

    @Override
    public void initialize() {
        command.cancel();
    }
}
