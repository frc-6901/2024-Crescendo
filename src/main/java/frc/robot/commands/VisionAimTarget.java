// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstans;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class VisionAimTarget extends PIDCommand {
  /** Creates a new VisionAimTarget. */
  public VisionAimTarget(VisionSubsystem camera, DriveSubsystem driveSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(VisionConstans.kVisionP, 0, VisionConstans.kVisionD),
        // This should return the measurement
        () -> camera.getLatestResult().getBestTarget().getYaw(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          driveSubsystem.drive(0, 0, output, false, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camera, driveSubsystem);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
