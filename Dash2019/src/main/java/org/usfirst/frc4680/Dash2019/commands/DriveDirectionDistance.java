/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.commands;

import org.usfirst.frc4680.Dash2019.Robot;
import org.usfirst.frc4680.Dash2019.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class DriveDirectionDistance extends Command {
  private double targetDirection;
  private double targetDistance;
  private final double tolerance = 1.0;
  private double sign;
  private double speed;

  public DriveDirectionDistance(double direction, double distance) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    targetDirection = direction;
    targetDistance = Robot.driveTrain.getDistance() + distance;
    sign = Math.signum(distance);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    speed = 0.5;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveTrain.directionDrive(sign * speed, targetDirection);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double dist = Robot.driveTrain.getDistance();
    return dist > targetDistance - tolerance && dist < targetDistance + tolerance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
