package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class IntakeRollerIOSparkFlex implements IntakeRollerIO {
  private final CANSparkFlex rollers;

  public IntakeRollerIOSparkFlex(int id) {
    rollers = new CANSparkFlex(id, MotorType.kBrushless);
    rollers.restoreFactoryDefaults();
    rollers.setIdleMode(IdleMode.kCoast);
    rollers.setInverted(true);

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT);
    rollers.setCANTimeout(250);
    rollers.burnFlash();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.rollerRotations = rollers.getEncoder().getPosition();
    inputs.rollerVelocityRPM = rollers.getEncoder().getVelocity();

    inputs.appliedVolts = rollers.getAppliedOutput() * 12.;
    inputs.currentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }
}
