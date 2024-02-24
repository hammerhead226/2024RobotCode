package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX falcon;

  private final StatusSignal<Double> feederVelocityRPS;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> feederRotations;

  private double velocitySetpointRPS = 0;

  public FeederIOTalonFX(int id) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FEEDER_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    falcon = new TalonFX(id, Constants.CANBUS);

    falcon.getConfigurator().apply(config);

    feederVelocityRPS = falcon.getVelocity();
    appliedVolts = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();
    feederRotations = falcon.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, feederVelocityRPS, appliedVolts, currentAmps, feederRotations);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(feederVelocityRPS, appliedVolts, currentAmps, feederRotations);

    inputs.feederRotations = feederRotations.getValueAsDouble();
    inputs.velocitySetpointRPS = velocitySetpointRPS;
    inputs.feederVelocityRPM = feederVelocityRPS.getValueAsDouble() * 60.;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void runCharacterization(double volts) {
    falcon.setVoltage(volts);
  }

  @Override
  public void setVelocityRPS(double velocityRPS, double ffVolts) {
    this.velocitySetpointRPS = velocityRPS;
    falcon.setControl(new VelocityVoltage(velocityRPS, 0, false, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    falcon.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs configs = new Slot0Configs();

    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;

    falcon.getConfigurator().apply(configs);
  }
}
