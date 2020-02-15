package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ControlPanelCMD extends SequentialCommandGroup {
    private static boolean state = false;

    public ControlPanelCMD(FSMReaderCmd fsm, RotateControlPanelCommand rotation_control, PositionControlPanelCommand position_control)
    {
        if(!state)
            addCommands(rotation_control);
        else
            addCommands(fsm,position_control);

        state = !state;
    }

}