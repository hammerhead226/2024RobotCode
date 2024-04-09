// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class AlignToNoteTele extends Command {
  /** Creates a new AlignToNote. */
  Intake intake;

  Pivot pivot;
  Shooter shooter;
  LED led;
  Drive drive;
  Command pathCommand;

  Translation2d noteTranslation2d;
  boolean pathReplanned = false;

  public AlignToNoteTele(Intake intake, Pivot pivot, Shooter shooter, Drive drive, LED led) {
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    this.led = led;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, intake, pivot, shooter, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteTranslation2d = drive.getCachedNoteLocation();
    this.pathCommand = drive.alignToNote(led);
    pathCommand.initialize();
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.seesNote()) {
      end(true);
    }
    pathCommand.execute();
    if (drive.getCachedNoteLocation().getDistance(noteTranslation2d) > Units.inchesToMeters(20)) {
      pathReplanned = true;
      replanPath();
    } else {
      pathReplanned = false;
    }

    Logger.recordOutput("Path Replanned", pathReplanned);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
    shooter.stopFeeders();
    pivot.setPivotGoal(Constants.PivotConstants.STOW_SETPOINT_DEG);
    pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.seesNote()) led.setState(LED_STATE.GREEN);
    return shooter.seesNote();
  }

  public void replanPath() {
    pathCommand.cancel();
    this.pathCommand = drive.alignToNote(led);
    pathCommand.initialize();
  }
}
