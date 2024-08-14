package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

public class AmpBarIOSIm implements AmpBarIO {

  private final DCMotor barGearBox = DCMotor.getNeo550(1);
  // private final LinearSystemSim sim =
  //   new Li

  private final PIDController pid = new PIDController(0, 0, 0);

  private double currentAmps = 0.0;
  private double appliedVolts = 0.0;
  private double velocityRPM = 0.0;
  private double positionRotations = 0.0;
  private double positionSetpointRotations = 0.0;

  @Override
  public void updateInputs(AmpBarIOInputs inputs) {
    positionSetpointRotations = pid.getSetpoint();

    // appliedVolts += MathUtil.clamp(pid.calculate(sim.get), velocityRPM, appliedVolts)

  }
}
