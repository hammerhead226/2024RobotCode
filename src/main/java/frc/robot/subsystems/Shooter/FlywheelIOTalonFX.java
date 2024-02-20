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

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX falcon;

  private final StatusSignal<Double> shooterVelocity;
  private final StatusSignal<Double> appliedAmps;
  private final StatusSignal<Double> currentAmps;
  private double velocitySetpoint = 0.0;

  public FlywheelIOTalonFX(int id) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    falcon = new TalonFX(id);

    falcon.getConfigurator().apply(config);

    shooterVelocity = falcon.getVelocity();
    appliedAmps = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, shooterVelocity, appliedAmps, currentAmps);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.shooterVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterVelocity.getValueAsDouble());

    inputs.appliedVolts = appliedAmps.getValueAsDouble();

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.velocitySetpoint = velocitySetpoint;
  }

  @Override
  public void setVelocity(double velocity, double ffVolts) {
    this.velocitySetpoint = velocity;
    falcon.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocity), 0, true, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    velocitySetpoint = 0;
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
