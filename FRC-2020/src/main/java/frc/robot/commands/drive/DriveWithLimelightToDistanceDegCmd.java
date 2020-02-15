/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartDashboardListener;
import frc.robot.commands.drive.pid.changeLimelightAngleDrivePID;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.Mechanum_Drivetrain;

public class DriveWithLimelightToDistanceDegCmd extends CommandBase {
  final double mm2in = 1.0 / 25.4;

  private final Mechanum_Drivetrain drive;
  private final Limelight_Subsystem limelight;

  private final double stopDist; // inches
  private double tolerancePct = .05;
  private double angleToleranceDeg = 3;
  private double kInchesToPerPower = -0.8;
  private double kDegreesToPerPower = 1;
  private double maxSpeed;
  private double angleTarget;
  private final double Kp = 0.2, Ki = 0.04, Kd = 0.25;
  private static double Kap = 0.1, Kai = 0.001, Kad = 0.0; //angle drive PIDs - static so changes to values will be retained
  private final PIDController distancePIDController;
  private final PIDController anglePIDController;
  private static NetworkTable networkTableSmartDashboard;
  private int listenerHandle; // may or may not be necessary for listener for pid changes

  /**
   * Creates a new DriveWithLidarToDistanceCmd.
   * 
   * stopDistance = inches to stop from the wall maxSpeed = percent max speed (+-
   * 1.0 max)
   * angleTarget = degrees from front of robot to target
   * Recommend using this command with withTimeout()
   * 
   * D Laufenberg
   * 
   */
  public DriveWithLimelightToDistanceDegCmd(final Mechanum_Drivetrain drive, final Limelight_Subsystem limelight,
      final double stopDist, final double angleTarget, final double maxSpeed) {
    this.drive = drive;
    this.limelight = limelight;
    this.stopDist = stopDist; //inches
    this.maxSpeed = maxSpeed;
    this.angleTarget = angleTarget;

    // create the PID with vel and accl limits
    distancePIDController = new PIDController(Kp, Ki, Kd);
    anglePIDController = new PIDController(Kap, Kai, Kad);


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
    addRequirements(drive);

    setPID();
    
    SmartDashboard.putData("Increase Limelight Angle Drive P", new changeLimelightAngleDrivePID('P', 0.01));
    SmartDashboard.putData("Decrease Limelight Angle Drive P", new changeLimelightAngleDrivePID('P', -0.01));
    SmartDashboard.putData("Increase Limelight Angle Drive I", new changeLimelightAngleDrivePID('I', 0.001));
    SmartDashboard.putData("Decrease Limelight Angle Drive I", new changeLimelightAngleDrivePID('I', -0.001));
    SmartDashboard.putData("Increase Limelight Angle Drive D", new changeLimelightAngleDrivePID('D', 0.001));
    SmartDashboard.putData("Decrease Limelight Angle Drive D", new changeLimelightAngleDrivePID('D', -0.001));
    
  }

  public DriveWithLimelightToDistanceDegCmd(Mechanum_Drivetrain drive, Limelight_Subsystem limelight, double stopDist, double maxSpeed, double angleTarget,
      double tolerancePct) {
    this(drive, limelight, stopDist, maxSpeed, angleTarget);
    this.tolerancePct = tolerancePct;
        
    setPID();

  }

  private void setPID() {
    SmartDashboard.putNumber("P", Kap); //TODO: ensure these work and values are editable - could use Shuffleboard.getTab(String).add(String, Object).withWidget(a widget).getEntry();
    SmartDashboard.putNumber("I", Kai);
    SmartDashboard.putNumber("D", Kad);
    networkTableSmartDashboard = NetworkTableInstance.getDefault().getTable("SmartDashboard"); //TODO ensure this is okay, and put in right place if applicable (unsure if method returns the reference or a clone)
    //TODO: addListener() may be needed
    int listenerHandle = networkTableSmartDashboard.addEntryListener(new SmartDashboardListener(), 0x20);
  }

  

  public static void setKap(double kap) {
    Kap = kap;
  }

  public static void setKai(double kai) {
    Kai = kai;
  }

  public static void setKad(double kad) {
    Kad = kad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.enableLED();
    distancePIDController.reset();
    distancePIDController.setSetpoint(stopDist);
    distancePIDController.setTolerance(stopDist * tolerancePct, 0.5);
    distancePIDController.setIntegratorRange(0, 3);

    anglePIDController.reset();
    anglePIDController.setSetpoint(angleTarget);
    anglePIDController.setTolerance(angleToleranceDeg, 0.5);



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // read Lidar range
    //double range = lidar.getAverageRange() * mm2in;
    //double speedCmd = distancePIDController.calculate(range) * kInchesToPerPower;
    double target_angle = limelight.getX();
    double angleCmd = kDegreesToPerPower * anglePIDController.calculate(target_angle);
    //speedCmd = MathUtil.clamp(speedCmd, -maxSpeed, maxSpeed);
    angleCmd = MathUtil.clamp(angleCmd, -maxSpeed, maxSpeed);

   /* SmartDashboard.putNumber("PID error (inches)", distancePIDController.getPositionError());    
    SmartDashboard.putNumber("PID Verr", distancePIDController.getVelocityError());
    SmartDashboard.putNumber("Range (inches)", range);
    SmartDashboard.putNumber("PID Output (%)", speedCmd);
*/

   // SmartDashboard.putNumber("PID error (degrees)", anglePIDController.getPositionError());
    SmartDashboard.putNumber("Angle", target_angle);
    SmartDashboard.putNumber("PID Output (%) (Angle)", angleCmd);

    

    Kap = networkTableSmartDashboard.getEntry("P").getDouble(Kap); //TODO: ensure values are editable
    Kai = networkTableSmartDashboard.getEntry("I").getDouble(Kai);
    Kad = networkTableSmartDashboard.getEntry("D").getDouble(Kad);

    SmartDashboard.putNumber("P", Kap); //TODO: These should be unnecessary if the network table and smart dashboard refer to the same object
    SmartDashboard.putNumber("I", Kap);
    SmartDashboard.putNumber("D", Kap);
    
    anglePIDController.setPID(Kap, Kai, Kad);
  

    SmartDashboard.putData(anglePIDController);

    // move forward, with rotation
    
    drive.driveCartesian(0.0, angleCmd, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    limelight.disableLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return distancePIDController.atSetpoint(); //test
return false;
  }
}
