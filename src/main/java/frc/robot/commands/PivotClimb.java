// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.statemachines.ClimbStateMachine;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotClimb extends SequentialCommandGroup {
  /** Creates a new PivotClimb. */
  public PivotClimb(ClimbStateMachine climbStateMachine, Elevator elevator, Pivot pivot) {

    // System.out.println("Test");
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorTarget(19., elevator), new SetPivotTarget(30, pivot));
    // climbStateMachine.advanceTargetState();
  }
}
