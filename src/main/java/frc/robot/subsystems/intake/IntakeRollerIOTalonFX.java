package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX falcon;

  private final StatusSignal<Angle> rollerRotations;
  private final StatusSignal<AngularVelocity> rollerVelocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  public IntakeRollerIOTalonFX(int id) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = Constants.IntakeConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    falcon = new TalonFX(id);

    falcon.getConfigurator().apply(config);

    rollerRotations = falcon.getPosition();
    rollerVelocityRPS = falcon.getVelocity();
    appliedVolts = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, rollerVelocityRPS, appliedVolts, currentAmps, rollerRotations);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(rollerRotations, rollerVelocityRPS, appliedVolts, currentAmps);

    inputs.rollerRotations = rollerRotations.getValueAsDouble();
    inputs.rollerVelocityRPM = rollerVelocityRPS.getValueAsDouble() * 60.;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    falcon.setVoltage(volts);
  }

  @Override
  public void stop() {
    falcon.stopMotor();
  }
}
