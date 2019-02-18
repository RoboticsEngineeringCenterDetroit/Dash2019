/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.commands;

import org.usfirst.frc4680.Dash2019.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Add your docs here.
 */
public class EjectBall extends TimedCommand {
  /**
   * Add your docs here.
   */
  public EjectBall(double timeout) {
    super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.manipulator.moveGrabber(1.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.manipulator.moveGrabber(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.manipulator.moveGrabber(0.0);
  }
}
