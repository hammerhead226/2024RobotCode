// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeRollerIO roller;

  private final IntakeRollerIOInputsAutoLogged rInputs = new IntakeRollerIOInputsAutoLogged();

  private final SimpleMotorFeedforward ffModel;

  public Intake(IntakeRollerIO roller) {
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0, 0);
        break;
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0, 0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0, 0.8);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0, 0);
        break;
    }

    this.roller = roller;

    // make this a constant
    roller.configurePID(0.5, 0, 0);
  }

  public void runRollers(double velocity) {
    roller.setVelocity(velocity, ffModel.calculate(velocity));
  }

  public void stopRollers() {
    roller.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roller.updateInputs(rInputs);

    Logger.processInputs("Intake", rInputs);
  }
}
