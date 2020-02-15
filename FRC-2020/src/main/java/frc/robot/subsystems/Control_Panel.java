/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Control_Panel extends SubsystemBase
{
    private static final int DEVICE_ID = 1;
    private static final int CHANNEL_A = 1;
    private static final int CHANNEL_B = 2;
    private static final int FORWARD_CHANNEL=3;
    private static final int REVERSE_CHANNEL=4;

    private TalonSRX m_talon;
    private DoubleSolenoid m_solenoid;
    private Encoder m_encoder;

    /*Initialization*/
    public Control_Panel(double distance_per_pulse, double minRate, double maxPeriod, int sampleToAverage)
    {
        m_talon = new TalonSRX(DEVICE_ID);
        m_solenoid = new DoubleSolenoid(FORWARD_CHANNEL,REVERSE_CHANNEL);
        /* Factory Default all hardware to prevent unexpected behaviour */
        m_talon.configFactoryDefault();
        m_talon.setInverted(false);
        m_talon.setNeutralMode(NeutralMode.Brake);
        //m_talon.configOpenloopRamp(0.2);
        //m_talon.configClosedloopRamp(0);
        m_encoder = new Encoder(CHANNEL_A,CHANNEL_B);
        m_encoder.setDistancePerPulse(distance_per_pulse);
        m_encoder.setMinRate(minRate);
        m_encoder.setMaxPeriod(maxPeriod);
        m_encoder.setSamplesToAverage(sampleToAverage);
    }

    @Override
    public void periodic() 
    {
      // This method will be called once per scheduler run
    }

    public void setSpeed(double x)
    {
         m_talon.set(ControlMode.Velocity,x);
    }


    public double getDistance()
    {
        return m_encoder.getDistance();
    }


    public void resetEncoder()
    {
        m_encoder.reset();
    }

    public void moveArm()
    {
        m_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractArm()
    {
        m_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    

    public void print()
    {
        SmartDashboard.putNumber("Distance", m_encoder.getDistance());
        SmartDashboard.putNumber("Distance per Pulse", m_encoder.getDistancePerPulse());
    }
}