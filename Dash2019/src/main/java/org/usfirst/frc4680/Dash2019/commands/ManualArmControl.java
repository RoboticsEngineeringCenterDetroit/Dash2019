/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.commands;

import org.usfirst.frc4680.Dash2019.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ManualArmControl extends Command {
  public ManualArmControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Robot.arm.enablePID(Robot.oi.armJoystick.getRawButton(5));

    double speed = Robot.oi.armJoystick.getRawAxis(1);

    if(Robot.arm.isPIDenabled()) {
      Robot.arm.moveShoulderSetpoint(speed);
    } else {
      Robot.arm.move(speed);
      Robot.arm.setAngle(Robot.arm.getAngle());
    }

    Robot.armExtender.enablePID(Robot.oi.armJoystick.getRawButton(6));

    double extensionSpeed = Robot.oi.armJoystick.getRawAxis(5);
    if(Robot.armExtender.isPIDenabled())
    {
      Robot.armExtender.moveExtensionSetpoint(extensionSpeed);
    } else {
      Robot.armExtender.move(extensionSpeed);
      Robot.armExtender.setLength(Robot.armExtender.getLength());
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
