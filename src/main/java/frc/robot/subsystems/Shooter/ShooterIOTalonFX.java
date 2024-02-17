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

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX falcon;

  private final StatusSignal<Double> shooterVelocity;
  private final StatusSignal<Double> appliedAmps;
  private final StatusSignal<Double> currentAmps;

  public ShooterIOTalonFX(int id) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.shooterCurrentLimits.TalonFXCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = Constants.shooterCurrentLimits.TalonFXCurrentLimitEnabled;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    falcon = new TalonFX(id);

    falcon.getConfigurator().apply(config);

    shooterVelocity = falcon.getVelocity();
    appliedAmps = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, shooterVelocity, appliedAmps, currentAmps);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterVelocity.getValueAsDouble());

    inputs.appliedVolts = appliedAmps.getValueAsDouble();

    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    falcon.setControl(new VelocityVoltage(velocity));
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
