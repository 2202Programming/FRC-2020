package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Color_Subsystem;

//TODO: NOT DONE
/**
 * Positions the control panel to an indicated color.
 */
public class PositionControlPanelCommand implements Command {
    private Color_Subsystem color_detector;
    private String init_color;
    private int init_color_index;
    private String final_color;
    private int final_color_index;
    private String[] colorOrder = null; //TODO: get color order of control panel (depending on how robot rotates panel), and should this be placed in Constants?

    public static final int NUM_DEGREES_PER_COLOR_SECTION = 45; // TODO: should this be placed in Constants?

    public PositionControlPanelCommand(Color_Subsystem color_detector, String final_color) {
        this.color_detector = color_detector;
        this.final_color = final_color;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        //Command.super.initialize();

        init_color = color_detector.getColor();
        if (init_color.equals(final_color))
            return; // TODO: take into account possibility of "Unknown" color(s)
        for (int i = 0; i < colorOrder.length; i++) {
            if (colorOrder[i].equals(init_color))
                init_color_index = i;
            if (colorOrder[i].equals(final_color))
                final_color_index = i;
        }
        int degreesToRotate = ((final_color_index - init_color_index + 4) % 4) * NUM_DEGREES_PER_COLOR_SECTION;
        //TODO: spin the control panel degreesToRotate degrees
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        //Command.super.execute();
    }

    @Override
    public boolean isFinished() {
        return color_detector.getColor().equals(final_color);
    }
    
}