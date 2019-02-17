/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
  public static final double MAX_LENGTH = 58.0;
  public static final double MAX_HORIZONTAL_LENGTH = 17.25 + 30.0;
  public static final double inchesPerEncoderCount = (15.0 * 0.375) / 4096.0;
  public static final double MIN_LENGTH = 22.5;


  private WPI_TalonSRX extensionMotor;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public ArmExtender() {
            
    extensionMotor = new WPI_TalonSRX(6);
    extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    extensionMotor.getSensorCollection().setQuadraturePosition(0, 10);
    extensionMotor.setName("extensionTalon");
    extensionMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void moveExtension(double speed) {
    extensionMotor.set(Math.signum(speed) * speed * speed);
  }

  public void stop() {
    extensionMotor.stopMotor();
  }

  public double getLength() {
    return MIN_LENGTH + (extensionMotor.getSensorCollection().getQuadraturePosition() * inchesPerEncoderCount);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ext Position", getLength());
  }

}

