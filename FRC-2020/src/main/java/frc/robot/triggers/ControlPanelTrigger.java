package frc.robot.triggers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlPanelTrigger extends Trigger
{

    private DigitalInput panel_switch;

    public ControlPanelTrigger(int channel)
    {
        panel_switch = new DigitalInput(channel);
    }

    @Override
    public boolean get() {
        // Gets the value of the digital input.  Returns true if the circuit is open.
        return panel_switch.get();
    }
}