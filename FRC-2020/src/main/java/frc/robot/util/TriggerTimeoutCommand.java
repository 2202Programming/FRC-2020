package frc.robot.util;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TriggerTimeoutCommand implements Command {
    private BooleanSupplier event;
    private double timeout;

    public TriggerTimeoutCommand(BooleanSupplier func, Double timeout) {
        event = func;
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        withTimeout(timeout);
    }

    @Override
    public boolean isFinished() {
        return event.getAsBoolean();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
