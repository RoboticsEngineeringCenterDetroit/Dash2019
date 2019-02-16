/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

// extender sprocket 16 teeth
// extender chain is #25 


public class ArmExtender extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final double inchesPerEncoderCount = (15.0 * 0.375) / 4096.0;

  private CANSparkMax extensionMotor;
  private CANEncoder encoder;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public ArmExtender() {
            
    extensionMotor = new CANSparkMax(6, MotorType.kBrushed);
    encoder = new CANEncoder(extensionMotor);
  }

  public void moveExtension(double speed) {
    extensionMotor.set(Math.signum(speed) * speed * speed);
  }

  public void stop() {
    extensionMotor.stopMotor();
  }

  public double getPosition() {
    return encoder.getPosition() * inchesPerEncoderCount;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ext Position", getPosition());
  }
}
