// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.VisionConstans;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionGetInRange extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final PIDController m_PIDController;

  /** Creates a new VisionGetInRange. */
  public VisionGetInRange(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_PIDController = new PIDController(VisionConstans.kVisionP, 0, VisionConstans.kVisionD);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_visionSubsystem.getLatestResult();

    if (result.hasTargets()) {
      double yaw = result.getBestTarget().getYaw();
      double output = m_PIDController.calculate(yaw, 0.0);

      m_driveSubsystem.drive(0, 0, output, false, false);
    }

    else {
      m_driveSubsystem.drive(0, 0, 0, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
