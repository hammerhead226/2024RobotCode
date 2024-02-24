package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotor simGearbox = DCMotor.getNeoVortex(1);
  private DCMotorSim sim = new DCMotorSim(simGearbox, 1, 0.7);
  private PIDController pid = new PIDController(0.2, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double velocitySetpoint = 0;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(Constants.LOOP_PERIOD_SECS);
    inputs.rollerVelocityRPM = sim.getAngularVelocityRPM();
    inputs.rollerVelocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.velocitySetpoint = velocitySetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocityRPM(double velocity, double ffVolts) {
    this.velocitySetpoint = velocity;
    closedLoop = true;
    pid.setSetpoint(velocity);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    setVelocityRPM(0, 0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
