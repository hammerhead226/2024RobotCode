package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotor simGearbox = DCMotor.getNeoVortex(1);
  private DCMotorSim sim = new DCMotorSim(simGearbox, 0.33, 0.1);
  private PIDController pid = new PIDController(0.2, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double velocitySetpointRPM = 0;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.rollerRotations = sim.getAngularPositionRotations();
    inputs.rollerVelocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.velocitySetpointRPM = velocitySetpointRPM;
  }

  @Override
  public void setVelocityRPM(double velocityRPM, double ffVolts) {
    this.velocitySetpointRPM = velocityRPM;
    closedLoop = true;
    pid.setSetpoint(velocityRPM);
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
