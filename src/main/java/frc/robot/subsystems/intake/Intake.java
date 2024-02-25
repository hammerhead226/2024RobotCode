// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeRollerIO roller;

  private final IntakeRollerIOInputsAutoLogged rInputs = new IntakeRollerIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD");

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA");

  private SimpleMotorFeedforward ffModel;

  public Intake(IntakeRollerIO roller) {
    switch (Constants.getMode()) {
      case REAL:
        kS.initDefault(0);
        kV.initDefault(10);
        kA.initDefault(0);

        kP.initDefault(0.03231);
        break;
      case REPLAY:
        kS.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0.03231);
        break;
      case SIM:
        kS.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0.03231);
        break;
      default:
        kS.initDefault(0);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0.03231);
        break;
    }

    this.roller = roller;

    // make this a constant
    roller.configurePID(kP.get(), 0, 0);
    ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
  }

  public void setRollerVelocityRPM(double velocityRPM) {
    roller.setVelocityRPM(velocityRPM, ffModel.calculate(velocityRPM));
  }

  public void stopRollers() {
    roller.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roller.updateInputs(rInputs);

    if (kP.hasChanged(hashCode())) {
      roller.configurePID(kP.get(), 0, 0);
    }

    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
      ffModel = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }

    Logger.processInputs("Intake", rInputs);
  }
}
