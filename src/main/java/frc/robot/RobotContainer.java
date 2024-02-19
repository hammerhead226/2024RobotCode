// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter.FeederIOSim;
import frc.robot.subsystems.Shooter.FeederIOTalonFX;
import frc.robot.subsystems.Shooter.FlywheelIOSim;
import frc.robot.subsystems.Shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter shooter;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(RobotMap.ShooterIDs.FLYWHEEL_ONE),
                new FlywheelIOTalonFX(RobotMap.ShooterIDs.FLYWHEEL_ONE),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER));
        break;
      case REPLAY:
        shooter = new Shooter(new FlywheelIOSim(), new FlywheelIOSim(), new FeederIOSim());
        break;
      case SIM:
        shooter = new Shooter(new FlywheelIOSim(), new FlywheelIOSim(), new FeederIOSim());
        break;
      default:
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(RobotMap.ShooterIDs.FLYWHEEL_ONE),
                new FlywheelIOTalonFX(RobotMap.ShooterIDs.FLYWHEEL_ONE),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER));
        break;
    }
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController
        .b()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setShooterVelocitys(1000, 1000),
                shooter::stopShooterMotors,
                shooter));
    m_driverController
        .a()
        .whileTrue(
            Commands.startEnd(() -> shooter.runFeeders(1000), shooter::stopFeeders, shooter));
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
