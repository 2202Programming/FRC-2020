package frc.robot.util.misc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CancelCommand extends InstantCommand {
    private Command command;

    public CancelCommand(Command c) {
        command = c;
    }

    @Override
    public void execute() {
        command.cancel();
    }
}
