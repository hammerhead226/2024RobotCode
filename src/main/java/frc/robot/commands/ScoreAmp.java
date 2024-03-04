// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmp extends SequentialCommandGroup {
  /** Creates a new ScoreAmp. */
  public ScoreAmp(Elevator elevator, Pivot pivot, Shooter shooter) {

    // new SetElevatorTarget(8, elevator)
    //             .andThen(new SetPivotTarget(Constants.PivotConstants.AMP_SETPOINT_DEG, pivot))
    //             .andThen(new InstantCommand(() -> shooter.setFlywheelRPMs(1200, 1200), shooter))
    //             .andThen(new InstantCommand(() -> shooter.setFeedersRPM(200), shooter)));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetElevatorTarget(8, elevator),
        new SetPivotTarget(Constants.PivotConstants.AMP_SETPOINT_DEG, pivot),
        // new WaitUntilCommand(pivot::atSetpoint),
        new WaitCommand(1),
        new InstantCommand(() -> shooter.setFlywheelRPMs(1200, 1200), shooter),
        new InstantCommand(() -> shooter.setFeedersRPM(200), shooter));
  }
}
