/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.commands;

import org.usfirst.frc4680.Dash2019.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class MoveArmTo extends Command   {
  double m_targetAngle;
  public MoveArmTo(double targetAngle) {
    m_targetAngle = targetAngle;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.setPosition(m_targetAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.arm.isAtSetpoint();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.arm.stop();
  }
}
