// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightShooterCanID, MotorType.kBrushless);
  private CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftShooterCanID, MotorType.kBrushless);

  private SparkPIDController m_rightMotorPIDController = m_rightMotor.getPIDController();
  private SparkPIDController m_leftMotorPIDController = m_leftMotor.getPIDController();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();

    m_rightMotor.setIdleMode(IdleMode.kCoast);
    m_leftMotor.setIdleMode(IdleMode.kCoast);

    m_rightMotorPIDController.setP(ShooterConstants.kShooterP);
    m_rightMotorPIDController.setI(ShooterConstants.kShooterI);
    m_rightMotorPIDController.setD(ShooterConstants.kShooterD);
    m_rightMotorPIDController.setIZone(ShooterConstants.kShooterIz);
    m_rightMotorPIDController.setFF(ShooterConstants.kShooterFF);
    m_rightMotorPIDController.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

    m_leftMotorPIDController.setP(ShooterConstants.kShooterP);
    m_leftMotorPIDController.setI(ShooterConstants.kShooterI);
    m_leftMotorPIDController.setD(ShooterConstants.kShooterD);
    m_leftMotorPIDController.setIZone(ShooterConstants.kShooterIz);
    m_leftMotorPIDController.setFF(ShooterConstants.kShooterFF);
    m_leftMotorPIDController.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
  }

  public void shoot() {
    // m_rightMotor.set(ShooterConstants.kShooterPower);
    // m_leftMotor.set(ShooterConstants.kShooterPower);

    m_rightMotorPIDController.setReference(ShooterConstants.kShooterRPM, ControlType.kVelocity);
    m_leftMotorPIDController.setReference(ShooterConstants.kShooterRPM, ControlType.kVelocity);
    }

  public void shootAmp() {
    m_rightMotor.set(ShooterConstants.kShooterAmpPower);
  }

  public void shooterIntake() {
    m_rightMotor.set(ShooterConstants.kShooterIntakePower);
  }

  public void stopShooter() {
    m_rightMotor.set(0);
  }

  public void increaseShooterPower() {
    ShooterConstants.kShooterPower += 0.05;
  }

  public void decreaseShooterPower() {
    ShooterConstants.kShooterPower -= 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Power", ShooterConstants.kShooterPower);

    SmartDashboard.putNumber("Right Shooter RPM", m_rightMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Left Shooter RPM", m_leftMotor.getEncoder().getVelocity());

    SmartDashboard.putNumber("Shooter P", ShooterConstants.kShooterP);
    SmartDashboard.putNumber("Shooter I", ShooterConstants.kShooterI);
    SmartDashboard.putNumber("Shooter D", ShooterConstants.kShooterD);
    SmartDashboard.putNumber("Shooter Iz", ShooterConstants.kShooterIz);
    SmartDashboard.putNumber("Shooter FF", ShooterConstants.kShooterFF);

    m_rightMotorPIDController.setP(SmartDashboard.getNumber("Shooter P", 0));
    m_rightMotorPIDController.setI(SmartDashboard.getNumber("Shooter I", 0));
    m_rightMotorPIDController.setD(SmartDashboard.getNumber("Shooter D", 0));
    m_rightMotorPIDController.setIZone(SmartDashboard.getNumber("Shooter Iz", 0));
    m_rightMotorPIDController.setFF(SmartDashboard.getNumber("Shooter FF", 0));

    m_leftMotorPIDController.setP(SmartDashboard.getNumber("Shooter P", 0));
    m_leftMotorPIDController.setI(SmartDashboard.getNumber("Shooter I", 0));
    m_leftMotorPIDController.setD(SmartDashboard.getNumber("Shooter D", 0));
    m_leftMotorPIDController.setIZone(SmartDashboard.getNumber("Shooter Iz", 0));
    m_leftMotorPIDController.setFF(SmartDashboard.getNumber("Shooter FF", 0));
  }
}
