// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PositionNoteInFeeder extends SequentialCommandGroup {
  /** Creates a new PositionNoteInFeeder. */
  public PositionNoteInFeeder(Shooter shooter, Intake intake) {

    // new InstantCommand(intake::stopRollers)
    //             .andThen(new InstantCommand(() -> shooter.setFeedersRPM(-100)))
    //             .andThen(
    //                 new WaitCommand(0.5)
    //                     .andThen(shooter::stopFeeders)
    //                     .andThen(
    //                         new SetPivotTarget(
    //                             Constants.PivotConstants.STOW_SETPOINT_DEG, pivot)))

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> shooter.setFeedersRPM(-400), shooter),
        new WaitCommand(0.723),
        new InstantCommand(shooter::stopFeeders, shooter));
  }
}
