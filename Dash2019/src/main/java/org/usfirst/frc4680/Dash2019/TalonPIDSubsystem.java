/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class TalonPIDSubsystem extends Subsystem  {

    protected TalonPIDController m_controller;
    protected PIDSourceTalon m_talon;

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

    public void stop() {
        if(m_controller.isEnabled() ) {
            m_controller.stop();
        } else {
            m_talon.stopMotor();
        }
    }

    public boolean onTarget() {
        return m_controller.onTarget();
    }

    public void move(double speed) {
        m_talon.set(Math.signum(speed) * speed * speed);
    }
    
    

    public abstract class TalonPIDController extends PIDController {
        public TalonPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
            super(p, i, d, f, src, out);
        }
        
        public void stop() {
          setSetpoint(m_pidInput.pidGet());
        }
      }
    

    

    public class PIDSourceTalon extends WPI_TalonSRX implements PIDSource {
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
    
        @Override
        public double pidGet() {
            return this.getSensorCollection().getQuadraturePosition();
        }
    
        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            // do nothing
        }
    
        public PIDSourceTalon(int CANid) {
            super(CANid);
            this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
            this.getSensorCollection().setQuadraturePosition(0, 10);
    
        }
    }
}
