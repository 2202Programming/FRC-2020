/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.kauailabs.navx.AHRSProtocol.AHRSUpdate;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.sim.AHRS_GyroSim;
import frc.robot.subsystems.ifx.Logger;
import frc.robot.subsystems.util.MonitoredSubsystemBase;

public class Sensors_Subsystem extends MonitoredSubsystemBase implements Gyro, Logger {

  public enum YawSensor { kNavX, kADXRS450, kBlended};
  

  /**
   * Creates a new Sensors_Subsystem.
   * 
   * This class will collect various robot sensors and ensure they are sampled and
   * filtered together.
   * 
   * Sensor sets include: NavX Chasis signals Lidar?
   * 
   * 
   */

  final double Kgyro = -1.0; // ccw is positive, just like geometry class

  private NetworkTable table;
  private NetworkTableEntry nt_accelX;
  private NetworkTableEntry nt_accelY;
  private NetworkTableEntry nt_accelZ;
  private NetworkTableEntry nt_yaw_navx;
  private NetworkTableEntry nt_yaw_navx_dot;
  private NetworkTableEntry nt_yaw_xrs450;
  private NetworkTableEntry nt_yaw_xrs450_dot;
  private NetworkTableEntry nt_yaw_blend;

  private NetworkTableEntry nt_canUtilization;
  private NetworkTableEntry nt_canTxError;
  private NetworkTableEntry nt_canRxError;

  static final byte update_hz = 100;
  // Sensors
  AHRS m_ahrs;
  Gyro m_gyro_ahrs;
  ADXRS450_Gyro m_gyro450;
  Gyro m_gyro;

  // CAN monitoring
  CANStatus m_canStatus;
  
  // Simulation
  AHRS_GyroSim m_gyroSim;

  //measured values
  double m_yaw_navx;
  double m_yaw_navx_d;
  double m_yaw_xrs450; 
  double m_yaw_xrs450_d;
  double m_yaw_blend;

  // configurion setting
  YawSensor c_yaw_type = YawSensor.kNavX;

  public Sensors_Subsystem() {

    // alocate sensors
    m_canStatus = new CANStatus();

    // create devices and interface access, use interface where possible
    m_gyro = m_gyro450 = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); 
    m_gyro_ahrs = m_ahrs = new AHRS(SPI.Port.kMXP, update_hz);
    m_ahrs.enableLogging(true);

    // setup network table
    table = NetworkTableInstance.getDefault().getTable("Sensors");
    nt_accelX = table.getEntry("x_dd");
    nt_accelY = table.getEntry("y_dd");
    nt_accelZ = table.getEntry("z_dd");
    
    nt_yaw_navx = table.getEntry("yaw_navx");
    nt_yaw_navx_dot = table.getEntry("yaw_navx_d");
    nt_yaw_xrs450 = table.getEntry("yaw_xrs450");
    nt_yaw_xrs450_dot = table.getEntry("yaw_xrs450_d");
    nt_yaw_blend = table.getEntry("yaw_blend");

    nt_canUtilization = table.getEntry("CanUtilization/value");
    nt_canRxError = table.getEntry("CanRxError");
    nt_canTxError = table.getEntry("CanTxError");

    calibrate();
    log();
  }

  public void setSensorType(YawSensor type ) {
    this.c_yaw_type = type;
  }

  @Override
  public void calibrate() {
    if ( m_gyro450.isConnected()) {
      m_gyro.calibrate();
    }

    if (m_ahrs.isConnected()) {
      m_ahrs.enableBoardlevelYawReset(true);
     
      m_ahrs.calibrate(); 
      System.out.print("\ncalibrating AHRS ");
      while (m_ahrs.isCalibrating()) { // wait to zero yaw if calibration is still running
        Timer.delay(0.25);
        System.out.print(".");
      }
      System.out.println(" done.");
      Timer.delay(0.1);
    }

    reset();
  }



  @Override
  public void monitored_periodic() {
    // This method will be called once per scheduler run
    m_yaw_navx = m_ahrs.getYaw();
    m_yaw_navx_d = m_ahrs.getRate();

    m_yaw_xrs450 = m_gyro450.getAngle();
    m_yaw_xrs450_d = m_gyro450.getRate();

    // simple average, but could become weighted estimator.
    m_yaw_blend = 0.5*(m_yaw_navx + m_yaw_xrs450);

    log();
  }

  void setupSimulation() {
   // m_gyroSim_ahrs = new AHRS_GyroSim(m_ahrs);
   // m_gyroSim SimDevice 
  }

  @Override
  public void simulationPeriodic() {
    //m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public void log() {
    nt_accelX.setDouble(m_ahrs.getWorldLinearAccelX());
    nt_accelY.setDouble(m_ahrs.getWorldLinearAccelY());
    nt_accelZ.setDouble(m_ahrs.getWorldLinearAccelZ());

    nt_yaw_navx.setDouble(m_yaw_navx);
    nt_yaw_navx_dot.setDouble(m_yaw_navx_d);

    nt_yaw_xrs450.setDouble(m_yaw_xrs450);
    nt_yaw_xrs450_dot.setDouble(m_yaw_xrs450_d);

    nt_yaw_blend.setDouble(m_yaw_blend);

    CANJNI.GetCANStatus(m_canStatus);
    nt_canUtilization.setDouble(m_canStatus.percentBusUtilization);
    nt_canRxError.setNumber(m_canStatus.receiveErrorCount);
    nt_canTxError.setNumber(m_canStatus.transmitErrorCount);
  }

  public void reset() {
    if ( m_gyro450.isConnected()) {
      m_gyro.reset();
    }

    if (m_ahrs.isConnected()) {
      m_ahrs.reset();
      m_ahrs.resetDisplacement();
    }
  }

  public double getYaw() {
    switch (c_yaw_type) {
      case kNavX:
        return m_yaw_navx;

      case kADXRS450:
        return m_yaw_xrs450;

      case kBlended:
      default:
        return m_yaw_blend;
    }
  }

 @Override
  public void close() throws Exception {
    m_gyro.close();
    m_gyro_ahrs.close();
  }


   /**
   * Return the heading of the robot in degrees.
   *
   * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>The angle is expected to increase as the gyro turns clockwise when looked at from the top.
   * It needs to follow the NED axis convention.
   *
   * <p>This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot in degrees.
   */
  @Override
  public double getAngle() {
    return getYaw();
  }


  /**
   * Return the rate of rotation of the gyro.
   *
   * <p>The rate is based on the most recent reading of the gyro analog value
   *
   * <p>The rate is expected to be positive as the gyro turns clockwise when looked at from the top.
   * It needs to follow the NED axis convention.
   *
   * @return the current rate in degrees per second
   */
  @Override
  public double getRate() {
    switch (c_yaw_type) {
      case kNavX:
        return m_yaw_navx_d;

      case kADXRS450:
        return m_yaw_xrs450_d;

      case kBlended:
      default:
        return 0.5*( m_yaw_navx_d +  m_yaw_xrs450_d);
    }
  }

/**
   * Return the heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   *
   * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * <p>This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot as a {@link
   *     edu.wpi.first.math.geometry.Rotation2d}.
   */
  @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-getAngle());
  }

  public static class Signals {
    public enum Signal {
      // WIP -
      T(0), X(1), Y(2), Xd(3), Yd(4), Xdd(5), Ydd(6), Roll(7), Pitch(8), Yaw(9), Roll_d(10), Pitch_d(11), Yaw_d(12),
      Roll_dd(13), Pitch_dd(14), Yaw_dd(15);

      public final int id;

      private Signal(int id) {
        this.id = id;
      }

      public int id() {
        return id;
      }

    }

    double data[] = new double[Signal.values().length];
  }

 

}
