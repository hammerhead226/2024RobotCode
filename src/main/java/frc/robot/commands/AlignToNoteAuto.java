// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class AlignToNoteAuto extends Command {
  /** Creates a new AlignToNote. */
  LED led;

  Drive drive;
  Pivot pivot;
  Intake intake;
  Shooter shooter;
  Command generatedPathCommand;
  Translation2d targetNoteLocation;

  private boolean finished;

  public AlignToNoteAuto(LED led, Drive drive, Shooter shooter, Intake intake, Pivot pivot) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.intake = intake;
    this.led = led;
    this.drive = drive;
    finished = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, shooter, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("auto pickup init", "true");

    led.setState(LED_STATE.FLASHING_RED);
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
    targetNoteLocation = drive.getTargetNoteLocation();

    generatedPathCommand =
        AutoBuilder.followPath(
            drive.generateTrajectoryToNote(targetNoteLocation, 3, 2.45, 100, 180, 0.5));

    generatedPathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    finished = shooter.seesNote() == NoteState.SENSOR;

    generatedPathCommand.execute();

    Logger.recordOutput("path is finished", finished);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setState(LED_STATE.BLUE);
    intake.stopRollers();
    shooter.stopFeeders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput("isFinished align note", shooter.seesNote());
    // return false;
    return shooter.seesNote() == NoteState.SENSOR
        || shooter.seesNote() == NoteState.CURRENT
        || finished;
  }
}
