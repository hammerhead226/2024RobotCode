package frc.robot.subsystems.shooter;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensorIORev implements DistanceSensorIO {
  private final Rev2mDistanceSensor distanceSensor;

  public DistanceSensorIORev() {
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    setAutomaticMode(true);
  }

  @Override
  public void updateInputs(DistanceSensorIOInputs inputs) {
    // TODO:: make constant
    inputs.hasNote = distanceSensor.getRange() <= 0;

    inputs.isConnected = distanceSensor.isEnabled();
  }

  @Override
  public void setAutomaticMode(boolean enabled) {
    distanceSensor.setAutomaticMode(enabled);
  }
}
