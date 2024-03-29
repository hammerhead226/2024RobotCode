// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class AlignToNote extends Command {
  /** Creates a new AlignToNote. */
  Intake intake;

  Pivot pivot;
  Shooter shooter;
  LED led;
  Drive drive;

  public AlignToNote(Intake intake, Pivot pivot, Shooter shooter, LED led, Drive drive) {
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    this.led = led;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.AlignToNote(intake, pivot, shooter, led);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.seesNote();
  }
}
