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
public interface VelocityDrive extends Subsystem {

    /**
     * Gets velocity from the drive train in physical or normalized units.
     * This should represent the drive trains best estimate of current speed.
     * Units is ft/s or normalized.
     *  
     * @param normalized  when true, returns vel in range of -1.0 to 1.0
     * @return
     */
    public double getLeftVel(boolean normalized);
    public double getRightVel(boolean normalized);
    public double getAvgVelocity(boolean normalized);
    
    // Position should be in physical units 
    public double getLeftPos();
    public double getRightPos();

    // resets the position counters on the drive train encoder
    public void resetPosition();

    /**
     * Commands to the drive train should be in physical units.
     * 
     * @param feetPerSecond
     * @param degreePerSecond
     */
    public void velocityArcadeDrive(double feetPerSecond, double degreePerSecond);
}