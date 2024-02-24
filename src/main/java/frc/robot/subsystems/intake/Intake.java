// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeRollerIO roller;

  private final IntakeRollerIOInputsAutoLogged rInputs = new IntakeRollerIOInputsAutoLogged();

  private static final LoggedTunableNumber intakekP = new LoggedTunableNumber("intake kP");
  private static final LoggedTunableNumber intakekD = new LoggedTunableNumber("intake kD");

  private final SimpleMotorFeedforward ffModel;

  private final SysIdRoutine sysId;

  public Intake(IntakeRollerIO roller) {
    switch (Constants.currentMode) {
      case REAL:
        ffModel = new SimpleMotorFeedforward(0, 0.003);
        intakekP.initDefault(0.0000000004);
        intakekD.initDefault(0.007);
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

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  roller.runCharacterization(voltage.in(Volts));
                },
                null,
                this));

    this.roller = roller;

    // make this a constant
    roller.configurePID(intakekP.get(), 0, 0);
  }

  public void runRollers(double velocity) {
    roller.setVelocityRPM(velocity, ffModel.calculate(velocity));
  }

  public void stopRollers() {
    roller.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roller.updateInputs(rInputs);

    if (intakekP.hasChanged(hashCode()) || intakekD.hasChanged(hashCode())) {
      roller.configurePID(intakekP.get(), 0, intakekD.get());
    }

    Logger.processInputs("Intake", rInputs);
  }
}
