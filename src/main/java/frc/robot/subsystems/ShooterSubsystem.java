// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightShooterCanID, MotorType.kBrushless);
  private CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftShooterCanID, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.restoreFactoryDefaults();

    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    // Follow the right motor inversly
    m_leftMotor.follow(m_rightMotor, true);
  }

  public void shoot() {
    m_rightMotor.set(ShooterConstants.kShooterPower);
  }

  public void shooterIntake() {
    m_rightMotor.set(-ShooterConstants.kShooterPower);
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
  }
}
