package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Color_Subsystem;
import frc.robot.subsystems.Control_Panel;

//TODO: NOT DONE
/**
 * Rotates the control panel an indicated number of times.
 */
public class RotateControlPanelCommand extends CommandBase {
    private static final double SPEED = 1;
    private static final int FULL_ROTATION = 360;
    private static final double STOP = 0;
    private static final double WHEEL_CIRCUMFERENCE = 42;//wrong
    private static final int PANEL_DIAMETER = 20;//20 inches
    private static final double PANEL_CIRCUMFERENCE = Math.PI * PANEL_DIAMETER;

    private Control_Panel panel;

    private int numRotationsNeeded;
    private int numSlices;
    private double degreesRotated;
    private String init_color;
    private String curr_color;
    private Color_Subsystem detector;
    //private String init_color;
    //private String final_color;
    //private String[] colorOrder = null; //TODO: get color order of control panel

    public RotateControlPanelCommand(int num_rotations,Control_Panel panel,Color_Subsystem detector) {
        numRotationsNeeded = num_rotations;
        degreesRotated = 0;
        this.detector = detector;
        this.panel = panel;
        numSlices = 25;
        //requirements
        addRequirements(panel);
        addRequirements(detector);
    }

    @Override
    public void initialize() {
        // TODO: Change
        //Command.super.initialize();
        init_color = detector.getColor();
        curr_color = init_color;
        //add arm
        panel.resetEncoder();
        panel.setSpeed(SPEED);
        
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        degreesRotated = findDegrees();
        if(!curr_color.equals(detector.getColor()))
            {
                numSlices--;
                curr_color = detector.getColor();
            }
    }

    @Override
    public boolean isFinished() {
        return degreesRotated >= numRotationsNeeded * FULL_ROTATION && numSlices == 0 && curr_color.equals(init_color); // 360 is number of degrees in one rotation
    }

    public double findDegrees()
    {
        return (((panel.getDistance()/FULL_ROTATION) * WHEEL_CIRCUMFERENCE)/PANEL_CIRCUMFERENCE)*FULL_ROTATION;
    }

    @Override
    public void end(boolean interrupted)
    {
        panel.setSpeed(STOP);
        //add arm
    }
    
}