// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static frc.robot.Constants.DT;
import static frc.robot.subsystems.Magazine_Subsystem.MAX_SOFT_STOP;
import static frc.robot.subsystems.Magazine_Subsystem.MIN_SOFT_STOP;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MagazineAngle extends CommandBase {
  // manual up/down at const motor speed or to an exact position
  public enum Direction {
    Up, Down, ToPositon
  };

  //static final double RPM = 3;
  static final double DEG_PER_SEC = 12.0;
  static final double DEG_PER_PERIOD = DEG_PER_SEC * DT;
  static final double CONFIRM_MOVE_DEG = 0.5;
  static final int    MIN_DOWN_COUNT = 3;
  static final int    BURP_COUNT = 10; // frames to let motor go down before giving up

  final Intake_Subsystem intake;
  final MagazinePositioner magPositioner;
  double cmd_degrees;
  Direction direction;
  ShooterSettings cmd_settings;

  // current state
  int m_delay;                // number of frames to try to unlock pawl, BURP_COUNT
  double m_init_angle;        // current angle at init of command
  double m_integrated_angle;  // counts up/down each frame if manual position
  double m_last_cmd_angle;    // track what we send to avoid repeated sending of angle
  boolean m_locked;           // lock stat at init of command
  boolean m_problem_detected; // couldn't unlock log and stop trying
  int m_down_count;
  boolean m_ok_to_lock;

  /** Creates a new MagazineAngle. */
  private  MagazineAngle(Intake_Subsystem intake){
    this.intake = intake;
    this.magPositioner = intake.getMagazine().getMagPositioner();
    this.cmd_settings = null;
    addRequirements(magPositioner);
  }

  public MagazineAngle(Intake_Subsystem intake, Direction dir) {
    this(intake);
    this.cmd_degrees = 0.0;
    this.direction = dir;
    this.cmd_settings = null;
  }

  public MagazineAngle(Intake_Subsystem intake, double degrees) {
    this(intake);
    this.cmd_degrees = degrees;
    this.direction = Direction.ToPositon;
  }

  public MagazineAngle(Intake_Subsystem intake, ShooterSettings settings) {
    this(intake);
    this.cmd_settings = settings;
    this.cmd_degrees = settings.angle;
    this.direction = Direction.ToPositon;
    addRequirements(magPositioner);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ok_to_lock = false;
    m_problem_detected = false;
    m_init_angle = magPositioner.get();
    // clamp the command in case our measured is a bit outside the soft limits
    m_integrated_angle =MathUtil.clamp(m_init_angle, MIN_SOFT_STOP, MAX_SOFT_STOP);
    m_locked = magPositioner.isLocked() || !magPositioner.getUnlockConfirmed();
    m_delay = 0;
    m_last_cmd_angle = -1;
    m_down_count = 0;

    magPositioner.unlock();
    
    // see if we need to burp the motor to unlock the pawl
    if (m_locked
        && ((direction == Direction.Up) || ((direction == Direction.ToPositon) && (m_init_angle <= cmd_degrees)))) {
      // we have to back up the motor and unlock
      magPositioner.burp();
     
      m_delay = BURP_COUNT;
    } else {
      // heading down, confirm the unlock right away
      magPositioner.unlockConfirmed();
    }
    // use the given settings or if null, a default will be used inside intake
    intake.setShooterSettings(cmd_settings);
  }

  @Override
  public void execute() {
    // handle unlocking with pawl
    if (m_delay > 0) {
        boolean unlocked = confirmUnlocked();
        // get out of here while still waiting
        if (unlocked == false) return;
    }

    // done looking for confirmation, magPositioner flag will be true if we are good.
    if (magPositioner.getUnlockConfirmed() == false) {
      // we have a problem
      System.out.println("ERROR unlocking the pawl.  No mag angle motor commands issued.");
      m_problem_detected = true;
      magPositioner.zeroPower(false);
      return;
    }

    // getting here, now we know we are unlocked, perform the commands
    // up/down will ramp up a position, ToPosition just sets the end goal.

    switch (direction) {
      case Up:  //magPositioner.wind(RPM);
        m_integrated_angle += DEG_PER_PERIOD;
        break;
      case Down: //magPositioner.wind(-RPM);
        m_integrated_angle -= DEG_PER_PERIOD;
        break;
      case ToPositon:
        m_integrated_angle = cmd_degrees;
        break;
    }
    
    //enforce soft limits
    m_integrated_angle = MathUtil.clamp(m_integrated_angle, MIN_SOFT_STOP, MAX_SOFT_STOP);

    //don't repeat postion commands
    if (m_last_cmd_angle == m_integrated_angle) 
      return;

      magPositioner.setAngle(m_integrated_angle);
    m_last_cmd_angle = m_integrated_angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // no matter how we got here, manual, or ToPosition, call it home.
    boolean lock = m_ok_to_lock && !interrupted;
    magPositioner.stopAndHold(lock);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_problem_detected)    return true;
    if ((direction == Direction.ToPositon) && 
         magPositioner.isAtSetpoint(cmd_degrees)) {
      m_ok_to_lock = true;
      return true;
    }
    return false; // manual never done
  }


  // looks at motor motion to know when it is safe to go up
  // if going down, it is ok because pawl will disengage
  boolean confirmUnlocked() {
    // check for motion
    if ((magPositioner.isMovingDown()) &&
       (m_down_count++ >= MIN_DOWN_COUNT)) {
        // moving down and far enough, unlock worked
        magPositioner.unlockConfirmed();
        m_delay = 0;
        System.out.println("*****burp count=" + m_down_count) ;
    } 
    // still waiting for movement and within BURP_COUNT
    m_delay--;
    return magPositioner.getUnlockConfirmed();
  }

}
