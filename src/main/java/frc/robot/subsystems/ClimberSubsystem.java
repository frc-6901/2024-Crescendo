// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private CANSparkMax m_RightClimber = new CANSparkMax(ClimberConstants.kRightClimberCanID, MotorType.kBrushless);
  private CANSparkMax m_LeftClimber = new CANSparkMax(ClimberConstants.kLeftClimberCanID, MotorType.kBrushless);
  
  private SparkLimitSwitch m_RightReverseLimit = m_RightClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
  private SparkLimitSwitch m_LeftReverseLimit = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_RightClimber.restoreFactoryDefaults();
    m_LeftClimber.restoreFactoryDefaults();

    m_RightClimber.setIdleMode(IdleMode.kBrake);
    m_LeftClimber.setIdleMode(IdleMode.kBrake);

    m_RightReverseLimit.enableLimitSwitch(true);
    m_LeftReverseLimit.enableLimitSwitch(true);

    SmartDashboard.putBoolean("Right Limit Switch", m_RightReverseLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Left Limit Switch", m_LeftReverseLimit.isLimitSwitchEnabled());
  }

  public void climb() {
    m_RightClimber.set(ClimberConstants.kClimberPower);
    m_LeftClimber.set(ClimberConstants.kClimberPower);
  }

  public void reverseClimb() {
    m_RightClimber.set(-ClimberConstants.kClimberPower);
    m_LeftClimber.set(-ClimberConstants.kClimberPower);  }

  public void stopClimb() {
    m_RightClimber.set(0);
    m_LeftClimber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_RightReverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Right Limit Switch", false));
    m_LeftReverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Left Limit Switch", false));

    SmartDashboard.putBoolean("Right Limit Switch", m_RightReverseLimit.isPressed());
    SmartDashboard.putBoolean("Left Limit Switch", m_LeftReverseLimit.isPressed());

    if (m_RightReverseLimit.isPressed()) {
      m_RightClimber.set(0);
    }

    if (m_LeftReverseLimit.isPressed()) {
      m_LeftClimber.set(0);
    }
  }
}
