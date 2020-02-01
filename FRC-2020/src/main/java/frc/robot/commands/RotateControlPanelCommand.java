package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Color_Subsystem;

//TODO: NOT DONE
/**
 * Rotates the control panel an indicated number of times.
 */
public class RotateControlPanelCommand extends CommandBase {
    private Color_Subsystem color_detector;
    private int numRotationsNeeded;
    private int degreesRotated = 0;
    //private String init_color;
    //private String final_color;
    //private String[] colorOrder = null; //TODO: get color order of control panel

    public RotateControlPanelCommand(Color_Subsystem color_detector, int num_rotations) {
        this.color_detector = color_detector;
        numRotationsNeeded = num_rotations;
    }

    @Override
    public void initialize() {
        // TODO: Change
        //Command.super.initialize();

        
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean isFinished() {
        return degreesRotated >= numRotationsNeeded * 360; // 360 is number of degrees in one rotation
    }
    
}