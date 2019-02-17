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

import org.usfirst.frc4680.Dash2019.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmExtender extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static final double MAX_LENGTH = 58.0;
  public static final double MIN_LENGTH = 22.5;
  public static final double MAX_HORIZONTAL_LENGTH = 17.25 + 30.0;
  public static final double LENGTH_TOLERANCE = 1;

  // 15 tooth sprocket, #35 chain has 3/8" pitch
  public static final double inchesPerEncoderCount = (15.0 * 0.375) / 4096.0;


  private PIDSourceTalon extensionMotor;
  private ArmExtenderPIDController m_controller;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public ArmExtender() {
            
    public static final double Kp = 0.02;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
    public static final double Kf = 0.0;

    extensionMotor = new PIDSourceTalon(6);
    extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    extensionMotor.getSensorCollection().setQuadraturePosition(0, 10);
    extensionMotor.setName("extensionTalon");
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    m_controller = new ArmExtenderPIDController(Kp, Ki, Kd, Kf, extensionMotor, extensionMotor);
  }

  public void moveExtensionSetpoint(double speed) {
    double setpoint = m_controller.getSetpoint();
    setpoint += (  speed / 5.0 );
    setLength(setpoint);
  }


  public void moveExtension(double speed) {
    extensionMotor.set(Math.signum(speed) * speed * speed);
  }

  public void stop() {
    if(m_controller.isEnabled() ) {
        m_controller.stop();
    } else {
        extensionMotor.stopMotor();
    }
}

  public double getLength() {
    return extensionMotor.getLength();
  }

  public void setLength(double len) {
    len = Math.max(len, getMinLength());
    len = Math.min(len, getMaxLength());
    m_controller.setSetpoint(len);
  }

  public double getMaxLength() {
    double angle_radians = Math.toRadians(Robot.arm.getAngle());
    double envelope_limit = MAX_HORIZONTAL_LENGTH * Math.cos(angle_radians);
    return Math.min(MAX_LENGTH, envelope_limit);
  }

  public double getMinLength() {
    //TODO make this based on arm angle
    return MIN_LENGTH;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ext Position", getLength());
    SmartDashboard.putNumber("Max Length", getMaxLength());
  }

  public class PIDSourceTalon extends WPI_TalonSRX implements PIDSource {
    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getLength();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // do nothing
    }

    PIDSourceTalon(int CANid) {
        super(CANid);
    }

    public double getLength() {
      return MIN_LENGTH + (extensionMotor.getSensorCollection().getQuadraturePosition() * inchesPerEncoderCount);
    }   
}


  public class ArmExtenderPIDController extends PIDController {
    ArmExtenderPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
          super(p, i, d, f, src, out);
      }

      //TODO override calculateFeedForward based on arm angle

      void stop() {
          setSetpoint(getLength());
      }
  }

  public void enablePID(boolean flag) {
    if(flag && !m_controller.isEnabled()) {
        m_controller.stop();
        m_controller.enable();
    }

    if(!flag && m_controller.isEnabled()) {
        m_controller.disable();
    }
  }

  public boolean isPIDenabled() {
    return m_controller.isEnabled();
  }

}

