package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX falcon;

  private final StatusSignal<Double> feederVelocity;
  private final StatusSignal<Double> appliedAmps;
  private final StatusSignal<Double> currentAmps;

  public FeederIOTalonFX(int id) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FEEDER_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    falcon = new TalonFX(id);

    falcon.getConfigurator().apply(config);

    feederVelocity = falcon.getVelocity();
    appliedAmps = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, feederVelocity, appliedAmps, currentAmps);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feederVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(feederVelocity.getValueAsDouble());

    inputs.appliedVolts = appliedAmps.getValueAsDouble();

    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity, double ffVolts) {
    falcon.setControl(new VelocityVoltage(velocity, 0, false, ffVolts, 0, false, false, false));
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
