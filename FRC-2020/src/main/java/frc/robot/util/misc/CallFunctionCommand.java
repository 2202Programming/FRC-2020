package frc.robot.util.misc;

import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CallFunctionCommand extends InstantCommand {
  IntSupplier workFunct;

  public CallFunctionCommand(IntSupplier workFunct) {
    this.workFunct = workFunct;
  }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        workFunct.getAsInt();
    }

}
