/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems.ifx;
import edu.wpi.first.wpilibj2.command.Subsystem;
/**
 *  All you should need for working with physical units and the drive train.
 */
public interface VelocityDrive extends Odometry, Subsystem {

    // allow some coasting if wanted
    public void setCoastMode(boolean coast);

    /**
     * Commands to the drive train should be in physical units.
     * 
     * @param lengthPerSecond
     * @param degreePerSecond
     */
    public void velocityArcadeDrive(double lengthPerSecond, double degreePerSecond);


    /** Some drive systems may have shifting, Expose the shifter if there is one. Null returned if none. */
    public Shifter getShifter();

}