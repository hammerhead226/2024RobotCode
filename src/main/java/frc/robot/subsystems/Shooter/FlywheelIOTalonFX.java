package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<Double> shooterVelocity;
  private final StatusSignal<Double> appliedAmps;
  private final StatusSignal<Double> currentAmps;
  private double velocitySetpoint = 0.0;

  public FlywheelIOTalonFX(int leadID, int followID) {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ShooterConstants.FLYWHEEL_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leader = new TalonFX(leadID, Constants.CANBUS);
    follower = new TalonFX(followID, Constants.CANBUS);

    leader.getConfigurator().apply(config);

    follower.setControl(new Follower(leadID, true));

    shooterVelocity = leader.getVelocity();
    appliedAmps = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, shooterVelocity, appliedAmps, currentAmps);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.shooterVelocity = shooterVelocity.getValueAsDouble();

    inputs.appliedVolts = appliedAmps.getValueAsDouble();

    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.velocitySetpoint = velocitySetpoint;
  }

  @Override
  public void setVelocity(double velocity, double ffVolts) {
    this.velocitySetpoint = velocity;
    leader.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocity), 0, true, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    velocitySetpoint = 0;
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs configs = new Slot0Configs();

    configs.kP = kP;
    configs.kI = kI;
    configs.kD = kD;

    leader.getConfigurator().apply(configs);
  }
}
