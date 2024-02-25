package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX falcon;

  private final StatusSignal<Double> rollerRotations;
  private final StatusSignal<Double> rollerVelocityRPS;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  private double velocitySetpointRPM = 0;

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
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.currentAmps = currentAmps.getValue();

    inputs.velocitySetpointRPM = velocitySetpointRPM;
  }

  @Override
  public void setVelocityRPM(double velocityRPM, double ffVolts) {
    this.velocitySetpointRPM = velocityRPM;

    falcon.setControl(new VelocityVoltage(velocityRPM, 0, false, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    this.velocitySetpointRPM = 0;
    falcon.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    falcon.getConfigurator().apply(config);
  }
}
