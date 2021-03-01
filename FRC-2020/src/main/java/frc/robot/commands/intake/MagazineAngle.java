// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static frc.robot.Constants.DT;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Intake_Subsystem.ShooterSettings;
import frc.robot.subsystems.Magazine_Subsystem.MagazinePositioner;

public class MagazineAngle extends CommandBase {
  // manual up/down at const motor speed or to an exact position
  public enum Direction {
    Up, Down, ToPositon
  };

  static final double RPM = 3;
  static final double DEG_PER_SEC = 4.0;
  static final double DEG_PER_PERIOD = DEG_PER_SEC * DT;
  static final double CONFIRM_MOVE_DEG = 0.2;
  static final int BURP_COUNT = 5; // frames to let motor go down before unlocking

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
    m_problem_detected = false;
    m_init_angle = magPositioner.get();
    m_integrated_angle = m_init_angle;
    m_locked = magPositioner.isLocked() || !magPositioner.getUnlockConfirmed();
    m_delay = 0;
    m_last_cmd_angle = -1;

    // see if we need to burp the motor to unlock the pawl
    if (m_locked
        && ((direction == Direction.Up) || ((direction == Direction.ToPositon) && (m_init_angle <= cmd_degrees)))) {
      // we have to back up the motor and unlock
      magPositioner.burp();
      magPositioner.unlock();
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
    magPositioner.stopAndHold(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_problem_detected)    return true;
    if (direction == Direction.ToPositon)  return magPositioner.isAtSetpoint(cmd_degrees);
    return false; // manual never done
  }


  // looks at motor motion to know when it is safe to go up
  // if going down, it is ok because pawl will disengage
  boolean confirmUnlocked() {
    // check for motion
    if ((magPositioner.isMovingDown() && 
        (m_init_angle - magPositioner.get()) > CONFIRM_MOVE_DEG)) {
      // moving down and far enough, unlock worked
      magPositioner.unlockConfirmed();
      m_delay = 0;
    } 
    // still waiting for movement and within BURP_COUNT
    m_delay--;
    return magPositioner.getUnlockConfirmed();
  }

}
