package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Color_Subsystem;
import frc.robot.subsystems.Control_Panel;

//TODO: NOT DONE
/**
 * Positions the control panel to an indicated color.
 */
public class PositionControlPanelCommand extends CommandBase {
    private static final int SLICE_DEGREE = 45;
    private static final double GEAR_RATIO= 3;
    private static final double RATE = 0.2;
    private static final int FULL_ROTATION = 360;
    private static final double STOP = 0;
    private static final double WHEEL_CIRCUMFERENCE = 4*Math.PI;//wrong
    private static final int PANEL_DIAMETER = 20;//20 inches
    private static final double PANEL_CIRCUMFERENCE = Math.PI * PANEL_DIAMETER;

    
    private Color_Subsystem color_detector;
    private double curr_speed;
    private double degreesRotated;
    private String init_color;
    private String curr_color;
    private String final_color;
    private int index_init;
    private int count;
    private int index_final;
    private Control_Panel panel;
    private String[] colors;

    public PositionControlPanelCommand(Color_Subsystem color_detector, Control_Panel panel) {
        this.color_detector = color_detector;
        this.panel = panel;
        curr_speed = 0.2;
        colors = new String[]{"Blue","Green","Red","Yellow"};
        degreesRotated = 0;
        //requirements
        addRequirements(color_detector);
        addRequirements(panel);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        //Command.super.initialize();
        init_color = color_detector.getColor();
        curr_color = init_color;
        final_color = panel.getTargetColor();
        for(int i = 0; i < colors.length; i++)
        {
            if(init_color.equals(colors[i]))
                index_init = i;
            if(final_color.equals(colors[i]))
                index_final = i;
        }
        panel.resetEncoder();
        panel.extendArm();
        panel.setSpeed(findShortest() * curr_speed);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        //Command.super.execute();
        if(!color_detector.getColor().equals(curr_color))
        {
            count--;
            curr_color = color_detector.getColor();
        }
        degreesRotated = findDegrees();
        ramp();
    }

    @Override
    public boolean isFinished() {
        return degreesRotated >= (findShortest()*SLICE_DEGREE) || (color_detector.getColor().equals(final_color) && count == 0);
    }

    public int findShortest()
    {
        if(index_init == 0 && index_final == 3)
            return -1;
        else if(index_init == 3 && index_final == 0)
            return 1;
        else if(index_init == index_final)
            return 0;
        else if(index_init > index_final)
            return -1;
        else 
            return 1;
    }

    public void findCount()
    {
        if(index_init == 0 && index_final == 3)
            count = 1;
        else if(index_init == 3 && index_final == 0)
            count = 1;
        else if(index_init == index_final)
            count = 0;
        else
            count = Math.abs(index_init-index_final);
    }

    public double findDegrees()
    {
        return ((((panel.getDistance()*GEAR_RATIO)/FULL_ROTATION) * WHEEL_CIRCUMFERENCE)/PANEL_CIRCUMFERENCE)*FULL_ROTATION;
    }

    @Override
    public void end(boolean interrupted)
    {
        panel.setSpeed(STOP);
        panel.retractArm();
    }

    public void ramp()
    {
        if(degreesRotated <= ((findShortest() * SLICE_DEGREE * FULL_ROTATION)/3))
        {
            curr_speed = ((curr_speed + findShortest() * RATE) <= 1)? curr_speed + findShortest() * RATE:findShortest() * 1;
            panel.setSpeed(curr_speed);
        }
        else if(degreesRotated >= ((2*findShortest() * SLICE_DEGREE * FULL_ROTATION)/3))
        {
            curr_speed = ((curr_speed - findShortest() * RATE) >= 0.2)? curr_speed - findShortest() * RATE:findShortest() * 0.2;
            panel.setSpeed(curr_speed);
        }
    }
    
}