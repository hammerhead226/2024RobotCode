// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorExtenderIO;
import frc.robot.subsystems.Elevator.ElevatorExtenderIOSim;
import frc.robot.subsystems.Elevator.ElevatorExtenderIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorPivotIO;
import frc.robot.subsystems.Elevator.ElevatorPivotIOSim;
import frc.robot.subsystems.Elevator.ElevatorPivotIOTalonFX;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LED_IO;
import frc.robot.subsystems.LED.LED_IOSpark;
import frc.robot.subsystems.Shooter.FeederIOSim;
import frc.robot.subsystems.Shooter.FeederIOTalonFX;
import frc.robot.subsystems.Shooter.FlywheelIOSim;
import frc.robot.subsystems.Shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSparkFlex;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private Shooter shooter;
  public Elevator elevator;
  private LED led;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        intake = new Intake(new IntakeRollerIOSparkFlex(RobotMap.IntakeIDs.ROLLERS));
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(
                    RobotMap.ShooterIDs.FLYWHEEL_ONE, RobotMap.ShooterIDs.FLYWHEEL_TWO),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER));
        elevator =
            new Elevator(
                new ElevatorPivotIOTalonFX(
                    RobotMap.ElevatorIDs.PIVOT_ONE,
                    RobotMap.ElevatorIDs.PIVOT_TWO,
                    RobotMap.ElevatorIDs.GYRO),
                new ElevatorExtenderIOTalonFX(
                    RobotMap.ElevatorIDs.EXTENDER_ONE, RobotMap.ElevatorIDs.EXTENDER_TWO));
        // led = new LED(new LED_IOSpark(RobotMap.LEDIDs.CHANNEL));
        break;
      case REPLAY:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new FeederIOSim());
        elevator = new Elevator(new ElevatorPivotIOSim(), new ElevatorExtenderIOSim());
        led = new LED(new LED_IOSpark(RobotMap.LEDIDs.CHANNEL));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new FeederIOSim());
        elevator = new Elevator(new ElevatorPivotIOSim(), new ElevatorExtenderIOSim());
        led = new LED(new LED_IOSpark(RobotMap.LEDIDs.CHANNEL));
        break;

      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeRollerIOSparkFlex(RobotMap.IntakeIDs.ROLLERS));
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(
                    RobotMap.ShooterIDs.FLYWHEEL_ONE, RobotMap.ShooterIDs.FLYWHEEL_TWO),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER));
        elevator = new Elevator(new ElevatorPivotIO() {}, new ElevatorExtenderIO() {});
        led = new LED(new LED_IO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Intake SysID Routines
    autoChooser.addDefaultOption(
        "Intake SysID (Dynamic Forward)", intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addDefaultOption(
        "Intake SysID (Dynamic Reverse)", intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addDefaultOption(
        "Intake SysID (Quasistatic Forward)",
        intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addDefaultOption(
        "Intake SysID (Dynamic Reverse)", intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.a().whileTrue(intake.sysIdQuasistatic(Direction.kForward));
    // controller.a().onTrue(new SetPivotTarget(20, elevator));
    // controller.a().onFalse(new InstantCommand(elevator::pivotStop, elevator));

    // controller.a().onTrue(new SetExtenderTarget(0, elevator));
    // controller.a().onFalse(new InstantCommand(elevator::elevatorStop, elevator));

    controller.a().onTrue(new InstantCommand(() -> shooter.setShooterVelocitys(300, 300), shooter));
    controller.a().onFalse(new InstantCommand(shooter::stopShooterMotors, shooter));

    controller.b().onTrue(new InstantCommand(() -> shooter.runFeeders(1500), shooter));
    controller.b().onFalse(new InstantCommand(shooter::stopFeeders, shooter));

    // controller.a().onTrue(new SetExtenderTarget(0, elevator));
    // controller.a().onFalse(new InstantCommand(elevator::elevatorStop, elevator));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // controller.a().whileTrue(new TurnToSpeaker(drive, controller));

    // controller.a().onTrue(new InstantCommand(() -> intake.runRollers(3000)));
    // controller.a().onFalse(new InstantCommand(() -> intake.stopRollers()));

    // controller.x().onTrue(new InstantCommand(() -> shooter.setShooterVelocitys(3000, 3000)));
    // controller.x().onFalse(new InstantCommand(() -> shooter.stopShooterMotors()));

    // controller.leftBumper().onTrue(Commands.run(() -> elevator.setExtenderGoal(1), elevator));
    // controller.rightBumper().onTrue(Commands.run(() -> elevator.setPivotGoal(Math.PI),
    // elevator));
    // controller.y().onTrue(Commands.run(() -> elevator.setPivotGoal(0), elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
