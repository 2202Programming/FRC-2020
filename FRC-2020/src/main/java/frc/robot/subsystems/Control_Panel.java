/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ExpoShaper;

public class Control_Panel extends SubsystemBase
{
    private static final double KEXPO = 0.5;
    private static final int TALON_CHANNEL = 1;

    private ExpoShaper shaper;
    private Talon m_talon;

    /*Initialization*/
    public Control_Panel()
    {
        shaper = new ExpoShaper(KEXPO);
        m_talon = new Talon(TALON_CHANNEL);
    }

    @Override
    public void periodic() 
    {
      // This method will be called once per scheduler run
    }

    public void setSpeed(double x)
    {
        m_talon.set(shaper.expo(x));
    }

    public void print()
    {
        
    }
}