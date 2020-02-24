package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class JoystickTrigger extends Button {
    private final GenericHID joystick;
    private final int axisNumber;
    private final double threshold;

    /**
     * Create a joystick trigger for triggering commands.
     *
     * @param joystick   The GenericHID object that has the button (e.g. Joystick,
     *                   KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     * 
     * @param threshold The value necessary for the trigger to activate (0-1) }     * 
     */
    public JoystickTrigger(GenericHID joystick, int axisNumber, double threshold) {
        this.joystick = joystick;
        this.axisNumber = axisNumber;
        this.threshold = threshold;
    }

    @Override
    public boolean get() {
        return joystick.getRawAxis(axisNumber) > threshold;
    }
}