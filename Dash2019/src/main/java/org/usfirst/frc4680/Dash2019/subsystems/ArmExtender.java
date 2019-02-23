/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.usfirst.frc4680.Dash2019.Robot;
import org.usfirst.frc4680.Dash2019.TalonPIDSubsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmExtender extends TalonPIDSubsystem {
  public static final double Kp = 0.02;
  public static final double Ki = 0.0;
  public static final double Kd = 0.0;
  public static final double Kf = 0.0;

  public static final double MAX_LENGTH = 50.0;
  public static final double MIN_LENGTH = 22;
  public static final double MAX_HORIZONTAL_LENGTH = 17.25 + 30.0;
  public static final double LENGTH_TOLERANCE = 1;

  // 15 tooth sprocket, #35 chain has 3/8" pitch
  public static final double inchesPerEncoderCount = (15.0 * 0.375) / 4096.0;

  public static double toInches(double encoderCounts) { return encoderCounts * inchesPerEncoderCount; }
  public static double toCounts(double inches) { return inches / inchesPerEncoderCount; }

  public ArmExtender() {
            
    m_talon = new PIDSourceTalon(6);
    m_talon.setName("extensionTalon");
    m_talon.setNeutralMode(NeutralMode.Brake);

    m_controller = new ArmExtenderPIDController(Kp, Ki, Kd, Kf, m_talon, m_talon);
    m_controller.setAbsoluteTolerance(LENGTH_TOLERANCE);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ext Position", getLength());
    SmartDashboard.putNumber("Max Length", getMaxLength());
  }

  public void moveExtensionSetpoint(double speed) {
    double pos = getLength();
    pos += (  speed / 5.0 );
    setLength(pos);
  }


  public double getLength() {
    return MIN_LENGTH + toInches(m_talon.pidGet());
  }

  public void setLength(double len) {
    len = Math.max(len, getMinLength());
    len = Math.min(len, getMaxLength());
    double counts = toCounts(len - MIN_LENGTH);
    m_controller.setSetpoint(counts);
  }

  public double getMaxLength() {
    double angle_radians = Math.toRadians(Robot.arm.getAngle());
    double envelope_limit = MAX_HORIZONTAL_LENGTH / Math.cos(angle_radians);
    return Math.min(MAX_LENGTH, envelope_limit);
  }

  public double getMinLength() {
    //TODO make this based on arm angle
    return MIN_LENGTH;
  }


  public class ArmExtenderPIDController extends TalonPIDController {
    ArmExtenderPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
          super(p, i, d, f, src, out);
      }

      @Override
      protected double calculateFeedForward() {
        double angle_radians = Math.toRadians(Robot.arm.getAngle());
        return getF() * Math.sin(angle_radians);
      }
  }


}

