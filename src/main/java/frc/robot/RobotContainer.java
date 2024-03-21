// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final CommandXboxController m_navigatorController =
      new CommandXboxController(ControllerConstants.kNavigatorPort);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(ControllerConstants.kOperatorPort);

  //private final VisionSubsystem m_vision = new VisionSubsystem();

  //private final ClimberSubsystem m_climb = new ClimberSubsystem();

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

    // Register PathPlanner Named Commands
    NamedCommands.registerCommand("shoot", Commands.runOnce(() -> m_shooter.shoot(), m_shooter));
    NamedCommands.registerCommand("stopShoot", Commands.runOnce(() -> m_shooter.stopShooter(), m_shooter));
    NamedCommands.registerCommand("intake", Commands.runOnce(() -> m_intake.intake(), m_intake));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    
    // Intake
    m_operatorController.rightBumper()
      .onTrue(Commands.runOnce(() -> m_intake.intake(), m_intake))
      .onFalse(Commands.runOnce(() -> m_intake.stopIntake(), m_intake));
    
    // Shoot into SPEAKER
    m_operatorController.leftBumper()
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

    // Reverse Intake
    m_operatorController.y()
      .onTrue(Commands.runOnce(() -> m_intake.outtake(), m_intake))
      .onFalse(Commands.runOnce(() -> m_intake.stopIntake(), m_intake));
    
    // // Climb
    // m_operatorController.x()
    //   .onTrue(Commands.runOnce(() -> m_climb.climb(), m_climb))
    //   .onFalse(Commands.runOnce(() -> m_climb.stopClimb(), m_climb));

    // // Reverse Climb
    // m_operatorController.a()
    //   .onTrue(Commands.runOnce(() -> m_climb.reverseClimb(), m_climb))
    //   .onFalse(Commands.runOnce(() -> m_climb.stopClimb(), m_climb));

    // // Stop Climber
    // m_operatorController.b().onTrue(Commands.runOnce(() -> m_climb.stopClimb(), m_climb));

    // Reset Gyro
    m_navigatorController.y().onTrue(m_robotDrive.zeroHeading());

    // Manual controls
    m_operatorController.povUp().onTrue(Commands.runOnce(() -> m_shooter.increaseShooterPower(), m_shooter));
    m_operatorController.povDown().onTrue(Commands.runOnce(() -> m_shooter.decreaseShooterPower(), m_shooter));

    m_operatorController.povRight().onTrue(Commands.runOnce(() -> m_intake.increaseIntakePower(), m_intake));
    m_operatorController.povLeft().onTrue(Commands.runOnce(() -> m_intake.decreaseIntakePower(), m_intake));
    
    // m_operatorController.povUp().onTrue(Commands.runOnce(() -> m_shooter.increaseShooterP(), m_shooter));
    // m_operatorController.povDown().onTrue(Commands.runOnce(() -> m_shooter.decreaseShooterP(), m_shooter));
    
    // m_operatorController.povRight().onTrue(Commands.runOnce(() -> m_shooter.increaseShooterD(), m_shooter));
    // m_operatorController.povLeft().onTrue(Commands.runOnce(() -> m_shooter.increaseShooterD(), m_shooter));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // return new PathPlannerAuto("Example Auto");
  //   return autoChooser.getSelected();
  // }

  // public Command getAutonomousCommand() {
  //   // SequentialCommandGroup MiddleShoot = new SequentialCommandGroup(
  //   //     m_robotDrive.zeroHeading(),
  //   //     new InstantCommand(
  //   //       () -> m_shooter.shoot(),
  //   //       m_shooter
  //   //     ),
  //   //     new InstantCommand(
  //   //       () -> m_shooter.stopShooter(),
  //   //       m_shooter
  //   //     ),
  //   //     new WaitCommand(10),
  //   //     new InstantCommand(
  //   //       () -> m_robotDrive.drive(0.2, 0, 0, false, false), 
  //   //       m_robotDrive),
  //   //     new WaitCommand(1.5),
  //   //     new InstantCommand(
  //   //       () -> m_robotDrive.drive(0, 0, 0, false, false),
  //   //       m_robotDrive
  //   //     )
  //   // );

  //   SequentialCommandGroup driveForward = new SequentialCommandGroup(
  //     new InstantCommand(
  //       () -> m_robotDrive.drive(0, 0.75, 0, false, false),
  //       m_robotDrive
  //     ),

  //     new WaitCommand(3),

  //     new InstantCommand(
  //       () -> m_robotDrive.drive(0, 0, 0, false, false)
  //     )
  //   );

  //   SequentialCommandGroup shootSpeaker = new SequentialCommandGroup(
  //     new InstantCommand(
  //       () -> m_shooter.shoot(),
  //       m_shooter
  //     ),

  //     new WaitCommand(2),

  //     new InstantCommand(
  //       () -> m_intake.intake(),
  //       m_intake
  //     ),

  //     new WaitCommand(1),

  //     new InstantCommand(
  //       () -> m_shooter.stopShooter(),
  //       m_shooter
  //     ),

  //     new InstantCommand(
  //       () -> m_intake.stopIntake(),
  //       m_intake
  //     )
  //   );
    
  //   // return shootSpeaker;
  //   // return driveForward;
  //   return null;
  //   }
  public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robotDrive.resetPose(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> m_robotDrive.stopModules()));
    }
}
