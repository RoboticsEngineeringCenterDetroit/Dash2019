/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.commands;

import org.usfirst.frc4680.Dash2019.Robot;
import org.usfirst.frc4680.Dash2019.Utility;
import org.usfirst.frc4680.Dash2019.subsystems.Arm;
import org.usfirst.frc4680.Dash2019.subsystems.ArmExtender;

import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.command.Command;

public class MoveArmTo extends Command   {
  double m_targetAngle;
  double m_targetLength;

  private static final double DELTA_ANGLE_MAX = (Arm.MAXIMUM_ANGLE - Arm.MINIMUM_ANGLE) / (4.0 * 50);
  private static final double DELTA_LENGTH_MAX = (ArmExtender.MAX_LENGTH - ArmExtender.MIN_LENGTH) / (4.0 * 50);

  public MoveArmTo(double targetAngle) {
    this(targetAngle, Robot.armExtender.getLength());
  }

  public MoveArmTo(double targetAngle, double targetLength) {
    m_targetAngle = targetAngle;
    m_targetLength = targetLength;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_targetAngle = Utility.clamp(m_targetAngle, Arm.MINIMUM_ANGLE, Arm.MAXIMUM_ANGLE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double angleDelta = m_targetAngle - Robot.arm.getAngle();
    angleDelta = Utility.clamp(angleDelta, -DELTA_ANGLE_MAX, DELTA_ANGLE_MAX);
    Robot.arm.setPosition(Robot.arm.getAngle() + angleDelta);

    double lenDelta = m_targetLength - Robot.armExtender.getLength();
    lenDelta = Utility.clamp(lenDelta, -DELTA_LENGTH_MAX, DELTA_LENGTH_MAX);
    Robot.armExtender.setLength(m_targetLength);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.arm.isAtSetpoint() && Robot.armExtender.isAtSetpoint();
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
    Robot.armExtender.stop();
  }

  
}
