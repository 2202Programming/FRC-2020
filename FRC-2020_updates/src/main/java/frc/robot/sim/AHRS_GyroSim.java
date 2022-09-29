package frc.robot.sim;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Add your docs here. */

/** Class to control a simulated ADXRS450 gyroscope. */
@SuppressWarnings({"TypeName", "AbbreviationAsWordInName"})
public class AHRS_GyroSim {
  final static String NAME = "Gyro:AHRS";
  final static String ANGLE = "angle_z";
  final static String RATE = "rate_z";

  AHRS device;
  SimDevice gyroSimDevice;

  final SimDouble m_simAngle;
  final SimDouble m_simRate;


  /**
   * Constructs from an AHRS_Gyro object.
   *
   * @param gyro AHRS_Gyro to simulate
   */
  public AHRS_GyroSim(AHRS device) {
    this.device = device;
    gyroSimDevice = SimDevice.create(NAME);
    gyroSimDevice.createDouble(ANGLE, Direction.kOutput, 0.00001);
    gyroSimDevice.createDouble(RATE, Direction.kOutput, 0.000);

    SimDeviceSim wrappedSimDevice = new SimDeviceSim(NAME);
    m_simAngle = wrappedSimDevice.getDouble(ANGLE);
    m_simRate = wrappedSimDevice.getDouble(RATE);
  }

  /**
   * Sets the angle in degrees (clockwise positive).
   *
   * @param angleDegrees The angle.
   */
  public void setAngle(double angleDegrees) {
    m_simAngle.set(angleDegrees);
  }

  /**
   * Sets the angular rate in degrees per second (clockwise positive).
   *
   * @param rateDegreesPerSecond The angular rate.
   */
  public void setRate(double rateDegreesPerSecond) {
    m_simRate.set(rateDegreesPerSecond);
  }
}
