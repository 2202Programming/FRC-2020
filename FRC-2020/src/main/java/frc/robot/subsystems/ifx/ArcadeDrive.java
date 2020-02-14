/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.ifx;
import edu.wpi.first.wpilibj2.command.Subsystem;
/**
 * Add your docs here.
 */
public interface ArcadeDrive extends Subsystem {
    public void arcadeDrive(double xSpeed, double zRot);
    public double getLeftVel(boolean normalized);
    public double getRightVel(boolean normalized);
    
    // Position should be in physical units
    public double getLeftPos();
    public double getRightPos();
    public void resetPosition();

    public void velocityArcadeDrive(double feetPerSecond, double degreePerSecond);

}
