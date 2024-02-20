package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import frc.robot.Constants;

public class FeederIOSparkMax implements FeederIO {
  private final CANSparkMax neo;
  private final SparkPIDController pid;


  public FeederIOSparkMax(int id) {

    neo = new CANSparkMax(id, MotorType.kBrushless);
    pid = neo.getPIDController();

    neo.restoreFactoryDefaults();
    neo.setSmartCurrentLimit((int) Constants.ShooterConstants.FEEDER_CURRENT_LIMIT);
    neo.setCANTimeout(250);
    neo.burnFlash();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {

    inputs.feederVelocity = neo.getEncoder().getVelocity();

    inputs.appliedVolts = neo.getBusVoltage();
    inputs.currentAmps = neo.getOutputCurrent();
  }

  @Override
  public void setVelocity(double velocity, double ffVolts) {
    pid.setReference(velocity, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    neo.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
