// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class AlignToNoteAuto extends Command {
  /** Creates a new AlignToNote. */
  LED led;

  Drive drive;
  Shooter shooter;
  Command pathCommand;

  private boolean finished;

  public AlignToNoteAuto(LED led, Drive drive, Shooter shooter) {
    this.shooter = shooter;
    this.led = led;
    this.drive = drive;
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive, shooter, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.pathCommand = drive.alignToNote();
    // pathCommand.schedule();
    drive.alignToNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("path is finished", pathCommand.isFinished());

    finished = shooter.seesNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setState(LED_STATE.PAPAYA_ORANGE);
    // pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput("isFinished align note", shooter.seesNote());
    return shooter.seesNote() || finished;
  }
}
