package frc.robot.commands.generic;

import java.util.function.IntSupplier;
import edu.wpi.first.wpilibj2.command.InstantCommand;
/**
 * Generic call a function as a instant command
 */
public class CallFunctionCmd extends InstantCommand {
  IntSupplier workFunct;

  /**
   *  IntSupplier used for function, int value is ignored
   */
  public CallFunctionCmd(IntSupplier workFunct) {
    this.workFunct = workFunct;
  }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void initialize() {
        workFunct.getAsInt();
    }

}
