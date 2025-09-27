// package frc.robot.subsystems.elevator;

// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.SparkPIDController.ArbFFUnits;
// import org.littletonrobotics.junction.Logger;

// public class AmpBarIOSparkMAX implements AmpBarIO {

//   private final CANSparkMax barMotor;
//   private final SparkPIDController pid;
//   private double barPositionSetpoint;
//   private final double gearRatio = 15. / 1.;

//   public AmpBarIOSparkMAX(int motorID) {

//     barMotor = new CANSparkMax(motorID, CANSparkLowLevel.MotorType.kBrushless);

//     pid = barMotor.getPIDController();
//     // pid.setFeedbackDevice(barMotor.getAbsoluteEncoder());

//     barMotor.restoreFactoryDefaults();
//     barMotor.setCANTimeout(250);
//     barMotor.setSmartCurrentLimit(30);
//     barMotor.getEncoder().setPosition(0);
//     barMotor.getEncoder().setPositionConversionFactor(1);
//     barMotor.setInverted(true);

//     barMotor.burnFlash();
//     barMotor.clearFaults();

//     barPositionSetpoint = 0;
//   }

//   @Override
//   public void updateInputs(AmpBarIOInputs inputs) {
//     inputs.barVelocityDegsPerSec = (barMotor.getEncoder().getVelocity() / gearRatio) * 360. / 60.;
//     inputs.barPositionDegrees = (barMotor.getEncoder().getPosition() / gearRatio) * 360.;
//     inputs.barPositionSetpointDegrees = barPositionSetpoint;
//     inputs.currentAmps = barMotor.getOutputCurrent();
//     inputs.appliedVolts = barMotor.getAppliedOutput();
//   }

//   @Override
//   public void setBrakeMode(boolean bool) {

//     if (bool) {
//       barMotor.setIdleMode(IdleMode.kBrake);
//     } else {
//       barMotor.setIdleMode(IdleMode.kCoast);
//     }
//   }

//   @Override
//   public void setPositionSetpoint(double barPositionOutputDegs, double ffVolts) {

//     this.barPositionSetpoint = barPositionOutputDegs;
//     Logger.recordOutput("barPositionOutputDegs", barPositionOutputDegs);
//     Logger.recordOutput("bar equation reference", (barPositionOutputDegs / 360.) * gearRatio);
//     Logger.recordOutput("ffVolts", ffVolts);
//     pid.setReference(
//         (barPositionOutputDegs / 360.) * gearRatio,
//         ControlType.kPosition,
//         0,
//         ffVolts,
//         ArbFFUnits.kVoltage);
//   }

//   @Override
//   public void setVoltage(double volts) {
//     barMotor.setVoltage(volts);
//   }

//   @Override
//   public void stop() {
//     barMotor.stopMotor();
//   }

//   @Override
//   public void configurePID(double kP, double kI, double kD) {

//     pid.setP(kP);
//     pid.setI(kI);
//     pid.setD(kD);
//   }
// }
