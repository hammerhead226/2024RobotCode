package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import frc.robot.Constants;

public class IntakeRollerIOSparkFlex implements IntakeRollerIO {
  private final CANSparkFlex rollers;
  private final SparkPIDController pid;

  private double velocitySetpointRPM = 0;

  public IntakeRollerIOSparkFlex(int id) {
    rollers = new CANSparkFlex(id, MotorType.kBrushless);
    rollers.restoreFactoryDefaults();
    rollers.setIdleMode(IdleMode.kCoast);
    rollers.setInverted(true);

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT);
    rollers.setCANTimeout(250);
    rollers.burnFlash();

    pid = rollers.getPIDController();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.rollerRotations = rollers.getEncoder().getPosition();
    inputs.rollerVelocityRPM = rollers.getEncoder().getVelocity();

    inputs.appliedVolts = rollers.getAppliedOutput();
    inputs.currentAmps = rollers.getOutputCurrent();
    inputs.velocitySetpointRPM = velocitySetpointRPM;
  }

  @Override
  public void setVelocityRPM(double velocity, double ffVolts) {
    this.velocitySetpointRPM = velocity;
    pid.setReference(velocity, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    this.velocitySetpointRPM = 0;
    rollers.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
