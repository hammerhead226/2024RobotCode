// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotIntakeTele extends SequentialCommandGroup {
  /** Creates a new PivotIntake. */
  public PivotIntakeTele(Pivot pivot, Intake intake, Shooter shooter, LED led, boolean outtake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (!outtake) {
      addCommands(
          // new InstantCommand(() -> led.setColor(LED_STATE.VIOLET)),
          new SetPivotTarget(Constants.PivotConstants.INTAKE_SETPOINT_DEG, pivot),
          new WaitUntilCommand(pivot::atSetpoint),
          new IntakeNote(intake, shooter),
          new InstantCommand(() -> led.setState(LED_STATE.FLASHING_GREEN)),
          new InstantCommand(() -> shooter.setFeedersRPM(-100)),
          new WaitCommand(0.4),
          new InstantCommand(shooter::stopFeeders),
          new WaitCommand(2),
          new InstantCommand(() -> led.setState(intake.getIntakeState())));
      // new InstantCommand(shooter::stopFeedWhenSeen, shooter));
      ;
    } else {
      addCommands(
          new SetPivotTarget(Constants.PivotConstants.INTAKE_SETPOINT_DEG, pivot),
          new WaitUntilCommand(pivot::atSetpoint),
          new InstantCommand(() -> shooter.setFeedersRPM(-4000)),
          new InstantCommand(
              () -> intake.runRollers(-Constants.IntakeConstants.APPLIED_VOLTAGE), intake));
    }
  }
}
