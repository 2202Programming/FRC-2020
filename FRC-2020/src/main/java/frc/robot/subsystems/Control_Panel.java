/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Control_Panel extends SubsystemBase {
    private WPI_TalonSRX rotationMotor = new WPI_TalonSRX(Constants.PANEL_ROTATION_CANID);
    // DistPerPulse: 1040, minRate: 10, maxPeriod: 50, sampleAverage: 20
    private DoubleSolenoid extensionSol = new DoubleSolenoid(Constants.PANEL_PISTON_FORWARD_PCM,
            Constants.PANEL_PISTON_REVERSE_PCM);
            
    /* Initialization */
    public Control_Panel() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        rotationMotor.configFactoryDefault();
        rotationMotor.setInverted(false);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // m_talon.configOpenloopRamp(0.2);
        // m_talon.configClosedloopRamp(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setSpeed(double speed) {
        rotationMotor.set(speed);
    }

    public double getDistance() {
        return rotationMotor.getSelectedSensorPosition();
    }

    public void resetEncoder() {
        rotationMotor.setSelectedSensorPosition(0);
    }

    public void extendArm() {
        extensionSol.set(DoubleSolenoid.Value.kForward);
    }

    public void retractArm() {
        extensionSol.set(DoubleSolenoid.Value.kReverse);
    }

    public String getTargetColor() {
        String fullGameData = DriverStation.getInstance().getGameSpecificMessage();
        String gameData = "";
        if (fullGameData.length() > 0) {
            switch (fullGameData.charAt(0)) {
            case 'B':
                // Blue case code
                gameData = "Blue";
                break;
            case 'G':
                // Green case code
                gameData = "Green";
                break;
            case 'R':
                // Red case code
                gameData = "Red";
                break;
            case 'Y':
                // Yellow case code
                gameData = "Yellow";
                break;
            default:
                // This is corrupt data
                gameData = "";
                break;
            }
        } else {
            // Code for no data received yet
            gameData = "";
        }

        return gameData;
    }

    public void log() {
        SmartDashboard.putNumber("Distance", getDistance());
    }
}