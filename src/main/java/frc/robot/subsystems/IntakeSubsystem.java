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
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_upper = new CANSparkMax(IntakeConstants.kUpperIntakeCanID, MotorType.kBrushless);
  private CANSparkMax m_lower = new CANSparkMax(IntakeConstants.kLowerIntakeCanID, MotorType.kBrushless);

  // IR sensor
  private SparkLimitSwitch m_noteLimit = m_lower.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_upper.restoreFactoryDefaults();
    m_lower.restoreFactoryDefaults();

    m_upper.setIdleMode(IdleMode.kBrake);
    m_lower.setIdleMode(IdleMode.kBrake);

    m_upper.setOpenLoopRampRate(0.5);
    m_lower.setOpenLoopRampRate(0.5);

    m_lower.follow(m_upper);

    //m_noteLimit.enableLimitSwitch(true);
    SmartDashboard.putBoolean("Note IR Sensor", m_noteLimit.isLimitSwitchEnabled());
  }

  public void intake() {
    m_upper.set(IntakeConstants.kIntakePower);
  }

  public void outtake() {
    m_upper.set(-IntakeConstants.kIntakePower);
}

  public void stopIntake() {
    m_upper.set(0);
  }

  public void increaseIntakePower() {
    IntakeConstants.kIntakePower += 0.05;
  }

  public void decreaseIntakePower() {
    IntakeConstants.kIntakePower -= 0.05;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    m_noteLimit.enableLimitSwitch(SmartDashboard.getBoolean("Note IR Sensor", false));
    SmartDashboard.putNumber("Intake Power", IntakeConstants.kIntakePower);

    if (m_noteLimit.isPressed()) {
     stopIntake();
    }
  }
}
