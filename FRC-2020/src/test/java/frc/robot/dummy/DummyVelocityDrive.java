/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dummy;

import frc.robot.subsystems.ifx.Shifter;
import frc.robot.subsystems.ifx.VelocityDrive;

/**
 * Add your docs here.
 */
public class DummyVelocityDrive implements VelocityDrive {

    @Override
    public double getLeftVel(boolean normalized) {
        return 0;
    }

    @Override
    public double getRightVel(boolean normalized) {
        return 0;
    }

    @Override
    public double getAvgVelocity(boolean normalized) {
        return 0;
    }

    @Override
    public double getLeftPos() {
        return 0;
    }

    @Override
    public double getRightPos() {
        return 0;
    }

    @Override
    public void resetPosition() {

    }

    @Override
    public void setCoastMode(boolean coast) {
    }

    @Override
    public void velocityArcadeDrive(double feetPerSecond, double degreePerSecond) {
    }

    @Override
    public Shifter getShifter() {
        return null;
    }

}
