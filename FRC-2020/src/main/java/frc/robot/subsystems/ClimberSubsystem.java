package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem that manages the climbing arm.
 * Manages lifting of robot and the folding of the arm.
 */
public class ClimberSubsystem extends SubsystemBase
{
    /** Initial knowledge of how the subsystem is to be run
     * 775 motor used for rotation(Talon controller)
     * neo motor used to shoot arm up(sparkmax controller)
     * mechanical stop for rotation
     * Ask Kevin for more info from Kevin
     */

    //Default constructor
    public ClimberSubsystem() 
    { 

    }

    @Override
    public void periodic() 
    {

    }
}