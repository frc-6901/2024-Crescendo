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
  private CANSparkMax m_climber = new CANSparkMax(ClimberConstants.ClimberCanID, MotorType.kBrushless);
  //private SparkLimitSwitch m_reverseLimit = m_climber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climber.restoreFactoryDefaults();
    m_climber.setIdleMode(IdleMode.kBrake);

    //m_reverseLimit.enableLimitSwitch(false);
    //SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
  }

  public void setClimb(double power) {
    m_climber.setVoltage(power);
  }

  public void reverseClimb() {
    ClimberConstants.ClimberPower *= -1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));
    //SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
  }
}
