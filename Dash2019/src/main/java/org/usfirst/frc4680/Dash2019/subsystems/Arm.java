// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc4680.Dash2019.subsystems;

import org.usfirst.frc4680.Dash2019.TalonPIDSubsystem;
import org.usfirst.frc4680.Dash2019.Utility;
import org.usfirst.frc4680.Dash2019.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Arm extends TalonPIDSubsystem {

    public static final double Kp_default = 0.01;
    public static final double Ki_default = 0.0;
    public static final double Kd_default = 0.0;
    public static final double Kf_default = 0.0;

    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    //Zero is with the arm straight horizontal
    public static final double MINIMUM_ANGLE = -21.0;
    public static final double MAXIMUM_ANGLE = 90.0;
    public static final double ANGLE_TOLERANCE = 2.0;
    public static final double STARTINGANGLE = 90.0;
    private static final double degreesPerEncoderCount = -(360.0 / 4096) * (12.0/60); 

    private WPI_TalonSRX pivotMotorB;

    public static double toDegrees(double counts) { return counts * degreesPerEncoderCount; }
    public static double toCounts(double degrees) { return degrees / degreesPerEncoderCount; }

    private NetworkTableEntry nt_Kp;
    private NetworkTableEntry nt_Ki;
    private NetworkTableEntry nt_Kd;
    private NetworkTableEntry nt_Kf;

    
    public Arm() {
       
        m_talon = new PIDSourceTalon(4);
        m_talon.setName("PivotTalonA");
        m_talon.setNeutralMode(NeutralMode.Brake);
 
        // NO ENCODER CONNECTED
        pivotMotorB = new WPI_TalonSRX(5);
        pivotMotorB.follow(m_talon); 
        pivotMotorB.setNeutralMode(NeutralMode.Brake);
        m_talon.setName("PivotTalonB");
               
        
        m_controller = new VerticalArmPIDController(Kp, Ki, Kd, Kf, m_talon, m_talon);
        m_controller.setOutputRange(-0.6, 0.6);
        m_controller.setAbsoluteTolerance(ANGLE_TOLERANCE);

        ShuffleboardTab pid_tab =  Shuffleboard.getTab("Arm PID");
        nt_Kd = pid_tab.add("kP",Kp_default).getEntry();
        nt_Ki = pid_tab.add("kI",Ki_default).getEntry();
        nt_Kd = pid_tab.add("kD",Kd_default).getEntry();
        nt_Kf = pid_tab.add("kF",Kf_default).getEntry();
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ManualArmControl());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putBoolean("Arm PID On", m_controller.isEnabled());
        SmartDashboard.putNumber("Arm error", toDegrees(m_controller.getError()));
//        SmartDashboard.putNumber("Arm Setpoint", toDegrees(m_controller.getSetpoint()));
        SmartDashboard.putNumber("Arm Setpoint", m_controller.getSetpoint());
        SmartDashboard.putNumber("Arm motor out", m_talon.get());

        Kp = nt_Kp.getDouble(0);
        Ki = nt_Ki.getDouble(0);
        Kd = nt_Kd.getDouble(0);
        Kf = nt_Kf.getDouble(0);
    }


    public void setAngle(double angle) {
        angle = Utility.clamp(angle, MINIMUM_ANGLE, MAXIMUM_ANGLE);
        double counts = toCounts(angle - STARTINGANGLE);
        m_controller.setSetpoint(counts);
    }

    public double getAngle() {
        int quadraturePosition = m_talon.getSensorCollection().getQuadraturePosition();
        return toDegrees(quadraturePosition) + STARTINGANGLE;
    }

    public void moveShoulderSetpoint(double speed) {
            double ang = getAngle();
            ang += (  speed / 2.0 );
            setAngle(ang);
    }


    public class VerticalArmPIDController extends TalonPIDController {
        VerticalArmPIDController(double p, double i, double d, double f, PIDSource src, PIDOutput out) {
            super(p, i, d, f, src, out);
        }

        @Override
        protected double calculateFeedForward() {
            return getF() * Math.cos(Math.toRadians(getAngle()));
        }
    }

}

