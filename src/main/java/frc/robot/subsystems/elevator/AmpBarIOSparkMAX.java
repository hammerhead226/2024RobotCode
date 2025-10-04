package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

public class AmpBarIOSparkMAX implements AmpBarIO {

  private final SparkMax barMotor;
  private final SparkClosedLoopController pid;
  private double barPositionSetpoint;
  private final double gearRatio = 15. / 1.;

  public AmpBarIOSparkMAX(int motorID) {

    barMotor = new SparkMax(motorID, SparkLowLevel.MotorType.kBrushless);

    pid = barMotor.getClosedLoopController();
    // pid.setFeedbackDevice(barMotor.getAbsoluteEncoder());

    barMotor.clearFaults();

    barPositionSetpoint = 0;
  }

  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    inputs.barVelocityDegsPerSec = (barMotor.getEncoder().getVelocity() / gearRatio) * 360. / 60.;
    inputs.barPositionDegrees = (barMotor.getEncoder().getPosition() / gearRatio) * 360.;
    inputs.barPositionSetpointDegrees = barPositionSetpoint;
    inputs.currentAmps = barMotor.getOutputCurrent();
    inputs.appliedVolts = barMotor.getAppliedOutput();
  }
}
