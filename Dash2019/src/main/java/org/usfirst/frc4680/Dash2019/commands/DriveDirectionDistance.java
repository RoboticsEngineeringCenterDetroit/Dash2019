/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.commands;

import org.usfirst.frc4680.Dash2019.Robot;
import org.usfirst.frc4680.Dash2019.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class DriveDirectionDistance extends Command {
  private final double DISTANCE_TOLERANCE = 1.0;
  private final double ACCEL_RAMP_TIME = 0.5;
  private final double DECEL_RAMP_TIME = 0.5;

  private double targetDirection;
  private double targetDistance;
  private double sign;
  private double speed;
  private boolean accelFlag;
  private double startTimeStamp;
  private double decelDistance;

  public DriveDirectionDistance(double direction, double distance) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    targetDirection = direction;
    targetDistance = Robot.driveTrain.getDistance() + distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    speed = 0.5;
    accelFlag = true;
    startTimeStamp = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    sign = Math.signum(distanceRemaining());
    
    if(accelFlag) {
      speed = Math.min(1.0, (elapsedTime()/ACCEL_RAMP_TIME));
      decelDistance = 0.5 * Robot.driveTrain.getSpeed() * DECEL_RAMP_TIME;
      if(distanceRemaining() <= decelDistance) {
        accelFlag = false;
      }
    } else {
      speed = Math.min(1.0, (distanceRemaining()/decelDistance));
    }

    Robot.driveTrain.directionDrive(sign * speed, targetDirection);
  }

  private double distanceRemaining() {
    return Robot.driveTrain.getDistance() - targetDistance;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double dist = Robot.driveTrain.getDistance();
    return dist > targetDistance - DISTANCE_TOLERANCE && dist < targetDistance + DISTANCE_TOLERANCE;
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

  public double elapsedTime() {
    return Timer.getFPGATimestamp() - startTimeStamp;
  }
}
