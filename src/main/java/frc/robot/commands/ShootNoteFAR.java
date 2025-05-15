// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteFAR extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNoteFAR(Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> shooter.setFlywheelRPMs(6000, 6000)),
        new WaitCommand(1),
        new InstantCommand(() -> shooter.setFeedersRPM(4000)),
        new WaitCommand(0.5),
        new InstantCommand(shooter::stopFeeders, shooter),
        new InstantCommand(shooter::stopFlywheels, shooter));
  }
}
