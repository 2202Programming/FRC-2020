/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import com.kauailabs.navx.AHRSProtocol.AHRSUpdate;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;

public class Sensors_Subsystem extends SubsystemBase {
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
  
   AHRS ahrs=null;
  
   public Sensors_Subsystem() {
    

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(SPI.Port.kMXP); 
      ahrs.reset();
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (ahrs != null) {
      //ahrs.
    }

  }


public static class Signals {
  public enum Signal {
    // 
    T(0),
    X(1),  Y(2), 
    Xd(3), Yd(4), 
    Xdd(5), Ydd(6), 
    Phi(7), Theta(8), Psi(9),
    Phid(10), Thetad(11), Psid(12),
    Phidd(13), Thetadd(14), Psidd(15)
    ;
     
    public final int id;
    private Signal(int id) { this.id = id; }
    public int id() {return id;}
  
  }
  double  data[] = new double[Signal.values().length];


}



}
