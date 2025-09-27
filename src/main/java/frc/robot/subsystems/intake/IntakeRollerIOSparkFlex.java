package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;

public class IntakeRollerIOSparkFlex implements IntakeRollerIO {
  private final SparkFlex rollers;

  public IntakeRollerIOSparkFlex(int id) {
    SparkMax rollers = new SparkMax(id, MotorType.kBrushless);
    SparkClosedLoopController pid = rollers.getClosedLoopController();
    
    
    

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT);
    rollers.setCANTimeout(250);

    rollers.SparkBase.PersistenMode.kPersistentParameters();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.rollerRotations = rollers.getEncoder().getPosition();
    inputs.rollerVelocityRPM = rollers.getEncoder().getVelocity();

    inputs.appliedVolts = rollers.getAppliedOutput() * 12.;
    inputs.currentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }
}
