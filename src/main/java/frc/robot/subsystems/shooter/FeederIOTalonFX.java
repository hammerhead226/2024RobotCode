package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX feeder;

  private final StatusSignal<AngularVelocity> feederVelocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;
  private final StatusSignal<Angle> feederRotations;

  private double velocitySetpointRPS = 0;

  public FeederIOTalonFX(int id) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FEEDER_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feeder = new TalonFX(id, Constants.CANBUS);

    feeder.getConfigurator().apply(config);

    feederVelocityRPS = feeder.getVelocity();
    appliedVolts = feeder.getMotorVoltage();
    currentAmps = feeder.getStatorCurrent();
    feederRotations = feeder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, feederVelocityRPS, appliedVolts, currentAmps, feederRotations);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(feederVelocityRPS, appliedVolts, currentAmps, feederRotations);

    inputs.feederRotations = feederRotations.getValueAsDouble();
    inputs.velocitySetpointRPM = velocitySetpointRPS * 60.;
    inputs.feederVelocityRPM = feederVelocityRPS.getValueAsDouble() * 60.;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setVelocityRPS(double velocityRPS, double ffVolts) {
    this.velocitySetpointRPS = velocityRPS;

    VelocityVoltage request =
        new VelocityVoltage(velocityRPS)
            .withAcceleration(0)
            .withEnableFOC(false)
            .withFeedForward(ffVolts)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(false)
            .withLimitForwardMotion(false)
            .withLimitReverseMotion(false);

    feeder.setControl(request);
  }

  @Override
  public void stop() {
    feeder.stopMotor();
    velocitySetpointRPS = 0;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs configs = new Slot0Configs();

    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;

    feeder.getConfigurator().apply(configs);
  }
}
