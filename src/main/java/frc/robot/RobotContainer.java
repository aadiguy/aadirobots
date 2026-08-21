// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.collector.CollectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final CollectorSubsystem collectorSubsystem = new CollectorSubsystem();
  public final Collector collector = new Collector(collectorSubsystem);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      public static final int RIGHT_DEPLOY_MOTOR = 1;
      public static final int LEFT_DEPLOY_MOTOR = 2;
      public static final int INTAKE_MOTOR = 3;
      public static final int BEAM_BRAKE  = 4;
      public static final int PIVOT_MOTOR = 5;
      public static final int FLYWHEEL_MOTOR = 6;
      public static final int PASSTHROUGH_MOTOR = 7;
      public static final int RIGHT_CLIMBER_MOTOR = 8;
      public static final int LEFT_CLIMBER_MOTOR = 9;
      public static final int ARM_MOTOR = 10;
      public static final int RELEASE_MOTOR = 11;
      public static final int FORWARD_BACKWARD_DRIVE = 12;
      public static final int LEFT_RIGHT_DRIVE = 13;
      public static final int FORWARD_BACKWARD_ENCODER = 14;
      public static final int LEFT_RIGHT_ENCODER = 15;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
     m_driverController.leftBumper().onTrue(
            Commands.either(
                collector.deployCollector(),
                collector.stowCollector(),
                collector.isStowing()
            )
        );

        m_driverController.leftTrigger()
            .whileTrue(collector.intakeCollector());

        m_driverController.a()
            .onTrue(collector.idleCollector());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
