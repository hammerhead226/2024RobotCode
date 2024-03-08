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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LED_STATE;
import frc.robot.commands.AlignToNoteAuto;
import frc.robot.commands.AutoPivotIntake;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PivotClimb;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.PivotSource;
import frc.robot.commands.PositionNoteInFeeder;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.SetElevatorTarget;
import frc.robot.commands.SetFeedersTargetRPM;
import frc.robot.commands.SetPivotTarget;
import frc.robot.commands.SetShooterTargetRPM;
import frc.robot.commands.ShootNoteAmp;
import frc.robot.commands.ShootNoteCenter;
import frc.robot.commands.ShootNoteSource;
import frc.robot.commands.StopIntakeFeed;
import frc.robot.statemachines.ClimbStateMachine;
import frc.robot.statemachines.ClimbStateMachine.CLIMB_STATES;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSparkFlex;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.shooter.DistanceSensorIO;
import frc.robot.subsystems.shooter.FeederIOSim;
import frc.robot.subsystems.shooter.FeederIOTalonFX;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Map;
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
  private Intake intake;
  private Shooter shooter;
  private Elevator elevator;
  private LED led;
  private Pivot pivot;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autos;
  private final ClimbStateMachine climbStateMachine;

  private CLIMB_STATES climbSelect() {
    return climbStateMachine.getTargetState();
  }

  private final Command climbCommands;

  private Command elevatorCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
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
                    RobotMap.ShooterIDs.FLYWHEEL_LEFT, RobotMap.ShooterIDs.FLYWHEEL_RIGHT),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER),
                new DistanceSensorIO() {});
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(RobotMap.ElevatorIDs.LEFT, RobotMap.ElevatorIDs.RIGHT));
        pivot =
            new Pivot(
                new PivotIOTalonFX(
                    RobotMap.PivotIDs.LEFT, RobotMap.PivotIDs.RIGHT, RobotMap.PivotIDs.GYRO));
        led = new LED(new LED_IOCANdle(20, Constants.CANBUS));
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
        shooter = new Shooter(new FlywheelIOSim(), new FeederIOSim(), new DistanceSensorIO() {});
        elevator = new Elevator(new ElevatorIOSim());
        pivot = new Pivot(new PivotIOSim());
        led = new LED(new LED_IOCANdle(20, Constants.CANBUS));
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new FeederIOSim(), new DistanceSensorIO() {});
        elevator = new Elevator(new ElevatorIOSim());
        pivot = new Pivot(new PivotIOSim());
        led = new LED(new LED_IO() {});
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
                    RobotMap.ShooterIDs.FLYWHEEL_LEFT, RobotMap.ShooterIDs.FLYWHEEL_RIGHT),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER),
                new DistanceSensorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIO() {});
        led = new LED(new LED_IO() {});
        break;
    }

    climbStateMachine = new ClimbStateMachine(elevator, shooter, pivot);

    climbCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    CLIMB_STATES.NONE,
                    new PivotClimb(climbStateMachine, elevator, pivot)
                        .andThen(climbStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    CLIMB_STATES.PIVOT_CLIMB,
                    new SetPivotTarget(Constants.PivotConstants.CLIMB_SETPOINT_ONE_DEG, pivot)
                        .andThen(new SetElevatorTarget(1, elevator))
                        .andThen(climbStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    CLIMB_STATES.RETRACT_CLIMB,
                    new SetPivotTarget(Constants.PivotConstants.CLIMB_SETPOINT_TWO_DEG, pivot)
                        .andThen(climbStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    CLIMB_STATES.SCORE_TRAP,
                    new SequentialCommandGroup(
                        new SetElevatorTarget(19, elevator),
                        new WaitCommand(1),
                        new SetPivotTarget(100, pivot))),
                Map.entry(CLIMB_STATES.DONE, new PrintCommand("hi"))),
            this::climbSelect);

    // climbCommands =
    //     new SelectCommand<>(
    //         Map.ofEntries(
    //             Map.entry(
    //                 CLIMB_STATES.NONE,
    //                 new SetPivotTarget(90, pivot)
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.PIVOT,
    //                 new SetElevatorTarget(19, elevator)
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.EXTEND,
    //                 new SetElevatorTarget(1, elevator)
    //                     .andThen(
    //                         new SetPivotTarget(
    //                             Constants.PivotConstants.CLIMB_SETPOINT_ONE_DEG, pivot))
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.RETRACT_CLIMB,
    //                 new SetPivotTarget(Constants.PivotConstants.CLIMB_SETPOINT_TWO_DEG, pivot)
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),

    //             // Trap Scoring Sequence
    //             Map.entry(
    //                 CLIMB_STATES.EXCHANGE_HOOK,
    //                 new SequentialCommandGroup(
    //                     new SetPivotTarget(90, pivot),
    //                     new WaitCommand(1),
    //                     new SetElevatorTarget(1, elevator),
    //                     new InstantCommand(climbStateMachine::advanceTargetState, pivot))),
    //             Map.entry(CLIMB_STATES.TRAP_STAGE_1, new SetPivotTarget(60, pivot)),
    //             Map.entry(
    //                 CLIMB_STATES.TRAP_STAGE_2,
    //                 new SequentialCommandGroup(
    //                     // new SetElevatorTarget(1, elevator),
    //                     // new WaitCommand(1),
    //                     new SetPivotTarget(85, pivot),
    //                     new InstantCommand(climbStateMachine::advanceTargetState, pivot))),
    //             Map.entry(
    //                 CLIMB_STATES.TRAP_STAGE_3,
    //                 new SequentialCommandGroup(
    //                     new SetElevatorTarget(19, elevator),
    //                     new WaitCommand(1),
    //                     new SetPivotTarget(108, pivot),
    //                     new InstantCommand(() -> shooter.setFlywheelRPMs(800, 800), shooter),
    //                     new InstantCommand(climbStateMachine::advanceTargetState, pivot))),

    //             // .andThen(
    //             //     new InstantCommand(() -> shooter.setFlywheelRPMs(800, 800), shooter))
    //             // .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.TRAP_STAGE_4,
    //                 new InstantCommand(() -> shooter.setFeedersRPM(800))
    //                     .andThen(new InstantCommand(climbStateMachine::advanceTargetState,
    // pivot))),
    //             Map.entry(
    //                 CLIMB_STATES.SHOOT,
    //                 new SequentialCommandGroup(
    //                     new InstantCommand(shooter::stopFeeders),
    //                     new InstantCommand(shooter::stopFlywheels),
    //                     new InstantCommand(climbStateMachine::advanceTargetState, pivot))),
    //             Map.entry(CLIMB_STATES.DONE, new PrintCommand("hi"))),
    //         this::climbSelect);

    elevatorCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    false,
                    new SetElevatorTarget(
                        Constants.ElevatorConstants.EXTEND_SETPOINT_INCH, elevator)),
                Map.entry(
                    true,
                    new SetElevatorTarget(
                        Constants.ElevatorConstants.RETRACT_SETPOINT_INCH, elevator))),
            elevator::isExtended);

    NamedCommands.registerCommand("StopIntakeFeed", new StopIntakeFeed(shooter, intake));

    NamedCommands.registerCommand(
        "PivotShoot", new SetPivotTarget(Constants.PivotConstants.SUBWOOFER_SETPOINT_DEG, pivot));

    NamedCommands.registerCommand("PivotShootFar", new SetPivotTarget(41, pivot));

    NamedCommands.registerCommand("PivotIntake", new PivotIntake(pivot, intake, shooter, false));
    NamedCommands.registerCommand(
        "AutoPivotIntake", new AutoPivotIntake(pivot, intake, shooter, 41, false));

    NamedCommands.registerCommand(
        "PivotSubwoofer",
        new SetPivotTarget(Constants.PivotConstants.SUBWOOFER_SETPOINT_DEG, pivot));

    NamedCommands.registerCommand("ShootNoteCenter", new ShootNoteCenter(shooter));
    // This one
    NamedCommands.registerCommand(
        "StartFlywheelsCenter",
        new InstantCommand(() -> shooter.setFlywheelRPMs(4000.0, 4000.0), shooter));

    NamedCommands.registerCommand(
        "StartFlywheelsSource", new InstantCommand(() -> shooter.setFlywheelRPMSSource(), shooter));

    NamedCommands.registerCommand(
        "StartFlywheelsAmp", new InstantCommand(() -> shooter.setFlywheelRPMSAmp(), shooter));

    NamedCommands.registerCommand("ShootNoteSource", new ShootNoteSource(shooter));

    NamedCommands.registerCommand("ShootNoteAmp", new ShootNoteAmp(shooter));

    NamedCommands.registerCommand(
        "StopFlywheels", new InstantCommand(shooter::stopFlywheels, shooter));

    NamedCommands.registerCommand(
        "StartFeeders", new InstantCommand(() -> shooter.setFeedersRPM(4000), shooter));

    NamedCommands.registerCommand("StopFeeders", new InstantCommand(shooter::stopFeeders, shooter));
    NamedCommands.registerCommand(
        "PositionNoteInFeeder", new PositionNoteInFeeder(shooter, intake));

    NamedCommands.registerCommand(
        "RunIntake",
        new InstantCommand(
            () -> intake.runRollers(Constants.IntakeConstants.APPLIED_VOLTAGE), intake));

    NamedCommands.registerCommand("AutoAlignNote", new AlignToNoteAuto(drive, led, 1.132));
    // NamedCommands.registerCommand("StopIntake", new
    // InstantCommand(intake::stopRollers, intake));

    // Set up auto routines
    autos = new SendableChooser<>();
    autos.addOption("hammertime", AutoBuilder.buildAuto("hammertime"));
    autos.addOption("4 piece auto align test", AutoBuilder.buildAuto("4 piece auto align test"));
    autos.addOption("c!p-b2", AutoBuilder.buildAuto("c!p-b2"));
    autos.addOption("s!p-b3-c4", AutoBuilder.buildAuto("s!p-b3-c4"));

    autos.addOption("s!p-c5", AutoBuilder.buildAuto("s!p-c5"));

    autos.addOption("a!p-c1", AutoBuilder.buildAuto("a!p-c1"));
    autos.addOption("a!p-b1-c2", AutoBuilder.buildAuto("a!p-b1-c2"));

    autos.addOption("b1 test auto", AutoBuilder.buildAuto("b1 test auto"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverControls();
    manipControls();

    // testControls();
  }

  private void testControls() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.getLeftTriggerAxis()));

    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driveController.rightBumper().onTrue(new PivotIntake(pivot, intake, shooter, false));
    driveController
        .rightBumper()
        .onFalse(
            new PositionNoteInFeeder(shooter, intake)
                .andThen(new InstantCommand(() -> intake.stopRollers(), intake))
                .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot)));

    driveController.leftBumper().whileTrue(new PivotIntake(pivot, intake, shooter, true));
    driveController
        .leftBumper()
        .onFalse(
            new InstantCommand(intake::stopRollers)
                .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
                .andThen(new InstantCommand(() -> shooter.stopFeeders())));

    // driveController.a().onTrue(new SetPivotTarget(40, pivot));
    driveController.a().onTrue(new SetElevatorTarget(19, elevator));

    //     driveController
    // .y()
    // .onTrue(new SetPivotTarget(Constants.PivotConstants.PODIUM_SETPOINT_DEG, pivot));

    // driveController.x().onTrue(new SetPivotTarget(108, pivot));

    driveController.x().onTrue(new PivotSource(pivot, intake, shooter, led));

    driveController
        .x()
        .onFalse(
            new InstantCommand(() -> shooter.setFeedersRPM(100), shooter)
                .andThen(new WaitCommand(0.5))
                .andThen(
                    new InstantCommand(shooter::stopFeeders, shooter)
                        .andThen(new InstantCommand(shooter::stopFlywheels, shooter))));
    //     driveController
    //         .x()
    //         .onTrue(new SetPivotTarget(Constants.PivotConstants.REVERSE_SUBWOOFER_SETPOINT_DEG,
    // pivot));

    // driveController
    // .rightTrigger()
    // .onTrue(new InstantCommand(() -> shooter.setFlywheelRPMs(4000, 4000)));
    driveController
        .rightTrigger()
        .onTrue(new InstantCommand(() -> shooter.setFlywheelRPMs(4000, 4000)));
    driveController.rightTrigger().onFalse(new InstantCommand(() -> shooter.stopFlywheels()));

    driveController.leftTrigger().onTrue(new InstantCommand(() -> shooter.setFeedersRPM(4000)));
    driveController.leftTrigger().onFalse(new InstantCommand(() -> shooter.stopFeeders()));

    // driveController.b().onTrue(new SetPivotTarget(56, pivot));
    // driveController.b().whileTrue(new AlignToNoteAuto(drive));
    //     driveController
    //         .b()
    //         .onFalse(
    //             DriveCommands.joystickDrive(
    //                 drive,
    //                 () -> -driveController.getLeftY(),
    //                 () -> -driveController.getLeftX(),
    //                 () -> -driveController.getRightX()));

    // driveController.a().onTrue(new SetPivotTarget(pivotAngle.get(), pivot));
    // driveController.a().onTrue(new SetPivotTarget(pivotAngle.get(), pivot));
    // driveController.a().onTrue(new InstantCommand(() ->
    // pivot.setPivotGoal(pivotAngle.get()),
    // pivot));
    driveController.a().onFalse(new InstantCommand(pivot::pivotStop, pivot));
    // driveController
    // .b()
    // .onTrue(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG,
    // pivot));

    // driveController
    // .x()
    // .onTrue(
    // new SetElevatorTarget(5.4, elevator)
    // .andThen(new InstantCommand(() -> shooter.setFlywheelRPMs(550, 550),
    // shooter)));
    // driveController
    // .x()
    // .onFalse(
    // new InstantCommand(() -> shooter.stopFlywheels(), shooter)
    // .andThen(new SetElevatorTarget(0, elevator)));
    // driveController.y().onTrue(new InstantCommand(() ->
    // shooter.setFeedersRPM(200), shooter));
    // driveController.y().onFalse(new InstantCommand(() -> shooter.stopFeeders(),
    // shooter));
    // driveController
    // .rightBumper()
    // .whileTrue(new TurnToSpeaker(drive, shooter, pivot, driveController));

    // THIS IS THE WORKING AMP CODE DO NOT DELETE PLEASE
    // driveController.y().onTrue(new ScoreAmp(elevator, pivot, shooter));
    // driveController
    //     .y()
    //     .onFalse(
    //         new InstantCommand(() -> shooter.stopFlywheels(), shooter)
    //             // .andThen(new SetElevatorTarget(10, elevator))
    //             .andThen(new SetElevatorTarget(0, elevator))
    //             .andThen(new InstantCommand(() -> shooter.stopFeeders(), shooter)));
  }

  private void driverControls() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.getLeftTriggerAxis()));

    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driveController.rightBumper().whileTrue(new PivotIntake(pivot, intake, shooter, false));
    driveController
        .rightBumper()
        .onFalse(
            new PositionNoteInFeeder(shooter, intake)
                .andThen(new InstantCommand(() -> intake.stopRollers(), intake))
                .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot)));

    driveController.leftBumper().whileTrue(new PivotIntake(pivot, intake, shooter, true));
    driveController
        .leftBumper()
        .onFalse(
            new InstantCommand(intake::stopRollers)
                .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
                .andThen(new InstantCommand(() -> shooter.stopFeeders())));

    driveController.rightTrigger().onTrue(new SetFeedersTargetRPM(1000, shooter));
    driveController
        .rightTrigger()
        .onFalse(new InstantCommand(() -> shooter.stopFeeders(), shooter));

    driveController.a().onTrue(climbCommands);

    driveController.x().onTrue(elevatorCommands);

    // driveController.y().onTrue(new ScoreAmp(elevator, pivot, shooter));
    // driveController
    //     .y()
    //     .onFalse(
    //         new InstantCommand(() -> shooter.stopFlywheels(), shooter)
    //             // .andThen(new SetElevatorTarget(10, elevator))
    //             .andThen(new SetElevatorTarget(0, elevator))
    //             .andThen(new InstantCommand(() -> shooter.stopFeeders(), shooter)));

    // driveController.leftTrigger().onTrue(new InstantCommand(drive::toggleLowSpeed, drive));
    // driveController.leftTrigger().onFalse(new InstantCommand(drive::toggleLowSpeed, drive));

    // driveController.y().onTrue(new InstantCommand(() ->
    // shooter.setFeedersRPM(200), shooter));
    // driveController.y().onFalse(new InstantCommand(() -> shooter.stopFeeders(),
    // shooter));
  }

  private void manipControls() {
    manipController.a().onTrue(new ScoreAmp(elevator, pivot, shooter));
    manipController
        .a()
        .onFalse(
            new InstantCommand(() -> shooter.stopFlywheels(), shooter)
                // .andThen(new SetElevatorTarget(10, elevator))
                .andThen(new SetElevatorTarget(0, elevator))
                .andThen(new InstantCommand(() -> shooter.stopFeeders(), shooter)));

    manipController
        .b()
        .onTrue(
            new ParallelCommandGroup(
                new SetPivotTarget(Constants.PivotConstants.SUBWOOFER_SETPOINT_DEG, pivot),
                new SetShooterTargetRPM(
                    Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
                    Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
                    shooter)));
    manipController
        .b()
        .onFalse(
            new ParallelCommandGroup(
                new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot),
                new SetShooterTargetRPM(0, 0, shooter)));

    manipController
        .leftBumper()
        .onTrue(new InstantCommand(() -> shooter.setFlywheelRPMs(3000, 3000), shooter));
    manipController.leftBumper().onFalse(new InstantCommand(shooter::stopFlywheels, shooter));

    manipController
        .y()
        .onTrue(
            new ParallelCommandGroup(
                new SetPivotTarget(Constants.PivotConstants.REVERSE_SUBWOOFER_SETPOINT_DEG, pivot),
                new SetShooterTargetRPM(
                    Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
                    Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
                    shooter)));
    manipController
        .y()
        .onFalse(
            new ParallelCommandGroup(
                new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot),
                new SetShooterTargetRPM(0, 0, shooter)));

    manipController.rightTrigger().onTrue(new SetFeedersTargetRPM(1000, shooter));
    manipController
        .rightTrigger()
        .onFalse(new InstantCommand(() -> shooter.stopFeeders(), shooter));

    manipController.x().onTrue(new PivotSource(pivot, intake, shooter, led));

    manipController
        .x()
        .onFalse(
            new InstantCommand(() -> led.setColor(LED_STATE.BLUE), led)
                .andThen(
                    new InstantCommand(() -> shooter.setFeedersRPM(100), shooter)
                        .andThen(new WaitCommand(0.5))
                        .andThen(
                            new InstantCommand(shooter::stopFeeders, shooter)
                                .andThen(new InstantCommand(shooter::stopFlywheels, shooter)))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Pivot getPivot() {
    return pivot;
  }

  public Intake getIntake() {
    return intake;
  }
}
