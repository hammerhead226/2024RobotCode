package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotor simGearbox = DCMotor.getNeoVortex(1);
  private DCMotorSim sim = new DCMotorSim(simGearbox, 0.33, 0.1);

 ;
  private PIDController pid = new PIDController(0.2, 0, 0);

  private boolean closedLoop = false;
  private double volts;
  private double appliedVolts;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM()) + volts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.rollerRotations = sim.getAngularPositionRotations();
    inputs.rollerVelocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    this.volts = volts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    setVoltage(0);
  }
}
