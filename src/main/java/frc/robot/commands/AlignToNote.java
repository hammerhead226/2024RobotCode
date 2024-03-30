// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class AlignToNote extends Command {
  /** Creates a new AlignToNote. */
  Intake intake;

  Pivot pivot;
  Shooter shooter;
  LED led;
  Drive drive;
  // Command pathCommand;
  int i;

  public AlignToNote(Intake intake, Pivot pivot, Shooter shooter, Drive drive) {
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    // this.led = led;
    this.drive = drive;
    i = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, intake, pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.pathCommand = drive.alignToNote();
    // pathCommand.schedule();
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
    drive.alignToNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("counter", i++);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
    shooter.stopFeeders();
    pivot.setPivotGoal(Constants.PivotConstants.STOW_SETPOINT_DEG);
    Logger.recordOutput("I HAVE ENDED", true);
    // pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput("i see a note!!!", shooter.seesNote());
    return shooter.seesNote();
  }
}
