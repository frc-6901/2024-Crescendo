// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final CommandXboxController m_navigatorController =
      new CommandXboxController(ControllerConstants.kNavigatorPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(ControllerConstants.kOperatorPort);

  //private final VisionSubsystem m_vision = new VisionSubsystem();

  private final ClimberSubsystem m_climb = new ClimberSubsystem();

  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final ShooterSubsystem m_shooter = new ShooterSubsystem();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_navigatorController.getLeftY(), ControllerConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_navigatorController.getLeftX(), ControllerConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_navigatorController.getRightX(), ControllerConstants.kDriveDeadband),
                    true, true),
                m_robotDrive));
    
    m_climb.setDefaultCommand(
      new RunCommand(
        () -> {
          m_climb.stopClimb();
        },
        m_climb)
    );

    // m_intake.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       m_intake.stopIntake();
    //     },
    //     m_intake)
    // );

    // m_shooter.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       m_shooter.stopShooter();
    //     },
    //     m_shooter)
    // );

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Configure your button bindings here
    //+m_navigatorController.b().onTrue(new VisionAimTarget(m_vision, m_robotDrive));

    // m_operatorController.rightBumper().onTrue(Commands.runOnce(() -> m_intake.setIntake(IntakeConstants.kUpperIntakePower, IntakeConstants.kLowerIntakePower), m_intake));
    m_operatorController.leftBumper().onTrue(Commands.runOnce(() -> m_intake.intake(), m_intake));

    // Shoot into SPEAKER
    m_operatorController.rightBumper()
      .onTrue(Commands.runOnce(() -> m_shooter.shoot(), m_shooter))
      .onFalse(Commands.runOnce(() -> m_shooter.stopShooter(), m_shooter));

    // COLLECTION Intake
    m_operatorController.leftTrigger()
      .onTrue(Commands.runOnce(() -> m_shooter.shooterIntake(), m_shooter))
      .onFalse(Commands.runOnce(() -> m_shooter.stopShooter(), m_shooter));

    // Shoot into AMP
    m_operatorController.rightTrigger()
      .onTrue(Commands.runOnce(() -> m_shooter.shootAmp(), m_shooter))
      .onFalse(Commands.runOnce(() -> m_shooter.stopShooter(), m_shooter));

    m_operatorController.x().onTrue(Commands.runOnce(() -> m_climb.climb(), m_climb));

    m_operatorController.a().onTrue(Commands.runOnce(() -> m_climb.reverseClimb(), m_climb));

    m_operatorController.b().onTrue(Commands.runOnce(() -> m_climb.stopClimb(), m_climb));

    m_navigatorController.y().onTrue(m_robotDrive.zeroHeading());

    // Manual controls
    m_operatorController.povUp().onTrue(Commands.runOnce(() -> m_shooter.increaseShooterPower(), m_shooter));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> m_shooter.decreaseShooterPower(), m_shooter));

    m_operatorController.povRight().onTrue(Commands.runOnce(() -> m_intake.increaseIntakePower(), m_intake));
    m_operatorController.povLeft().onTrue(Commands.runOnce(() -> m_intake.decreaseIntakePower(), m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *Sam is a weird monkey :) ()
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(
      "MiddleNode1");
    //MiddleNode1
    //RightNode1
    //LeftNode1
    //Move
    //MiddleFar
    //LeftAmp
    //Right2Nodes
  }
}
