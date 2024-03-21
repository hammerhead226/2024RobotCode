// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_STATE;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeRollerIO roller;

  private final IntakeRollerIOInputsAutoLogged rInputs = new IntakeRollerIOInputsAutoLogged();

  private boolean isAutoAlign;

  public Intake(IntakeRollerIO roller) {
    this.roller = roller;

    isAutoAlign = false;
  }

  public boolean isAutoAlign() {
    return isAutoAlign;
  }

  public void toggleAutoAlign() {
    this.isAutoAlign = !this.isAutoAlign;
  }

  public void runRollers(double volts) {
    roller.setVoltage(volts);
  }

  public void stopRollers() {
    roller.stop();
  }

  public LED_STATE getIntakeState() {
    return isAutoAlign() ? LED_STATE.AUTO_ALIGN : LED_STATE.NORMAL_INTAKE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roller.updateInputs(rInputs);

    Logger.processInputs("Intake", rInputs);
  }
}
