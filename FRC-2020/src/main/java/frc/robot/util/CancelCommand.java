package frc.robot.commands.util;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class CancelCommand extends InstantCommand {
    private Command command;

    public CancelCommand(Command c) {
        command = c;
    }

    @Override
    protected void execute() {
        command.cancel();
    }
}
