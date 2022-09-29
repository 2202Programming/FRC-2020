// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Add your docs here. */
public class EncoderSim2 {
  CANSparkMax controller;
  SimDevice encoder;
  SimDouble m_simVelocity;
  SimDouble m_simDistance;

  final static String VEL = "velocity";
  final static String POS = "position";

  public EncoderSim2( CANSparkMax controller) {
    this.controller = controller;
    encoder = SimDevice.create("CANSparkMax", controller.getDeviceId());
    encoder.createDouble(VEL, Direction.kOutput, 0.0001);
    encoder.createDouble(POS, Direction.kOutput, 0.0002);

    SimDeviceSim wrappedSimDevice = new SimDeviceSim("CANSparkMax" + "[" + controller.getDeviceId() + "]");
    m_simVelocity = wrappedSimDevice.getDouble(VEL);
    m_simDistance = wrappedSimDevice.getDouble(POS);
  }

  public void setRate(double vel) {
    m_simVelocity.set(vel);
  }

  public void setDistance(double dist) {
    m_simDistance.set(dist);
  }

}
