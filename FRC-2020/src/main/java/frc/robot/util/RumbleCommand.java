package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.Set;
import java.util.function.BooleanSupplier;

public class RumbleCommand implements Command {
    final double kRumbleTime = 2.300; // seconds - 300 ms to make noise

    BooleanSupplier boolFunc;
    XboxController ctrlr;
    double m_startTime;
    Trigger vButton;

    public RumbleCommand(XboxController ctrlr, BooleanSupplier boolFunct) {
        this.boolFunc = boolFunct;
        this.ctrlr = ctrlr;

        // tie the boolFunc to a trigger
        vButton = new RumbleTrigger(boolFunct);
        vButton.whenActive(this);
    }

    /**
     * Zeros the arm's encoders - arm and extension are at starting point
     */
    @Override
    public void initialize() {
        withTimeout(kRumbleTime);
        ctrlr.setRumble(RumbleType.kRightRumble, 0.1);
        ctrlr.setRumble(RumbleType.kLeftRumble, 0.1);
        System.out.print("**********RUMBLE RUMBLE RUMBLE******");
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return boolFunc.getAsBoolean();
    }

    protected void end() {
        ctrlr.setRumble(RumbleType.kRightRumble, 0.0);
        ctrlr.setRumble(RumbleType.kLeftRumble, 0.0);
        System.out.print("N0 RUMBLE No RUMBLE n0 RUMBLE");
    }

    // Trigger to tie to this command
    public class RumbleTrigger extends Trigger {
        BooleanSupplier boolFunc;

        public RumbleTrigger(BooleanSupplier boolFunct) {
            this.boolFunc = boolFunct;
        }

        @Override
        public boolean get() {
            return boolFunc.getAsBoolean();
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
