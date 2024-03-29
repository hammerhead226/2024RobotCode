// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickupNote extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  private final Drive drive;

  private Rotation2d targetRotation = new Rotation2d();

  public AutoPickupNote(Intake intake, Pivot pivot, Shooter shooter, Drive drive, LED led) {

    this.drive = drive;

    addCommands();
    // new InstantCommand(() ->
    // pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG)),
    // new InstantCommand(() -> intake.runRollers(12)),
    // new InstantCommand(() -> shooter.setFeedersRPM(500)),
    // new InstantCommand(() -> led.setState(LED_STATE.FLASHING_BLUE)),
    // drive.AlignToNote(led));
  }
}
