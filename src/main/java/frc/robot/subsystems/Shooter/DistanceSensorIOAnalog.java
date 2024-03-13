package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class DistanceSensorIOAnalog implements DistanceSensorIO {
    private final AnalogInput proxSensor;
    private int sustain;

    public DistanceSensorIOAnalog() {
        this.proxSensor = new AnalogInput(0);
        sustain = 0;
    }

    @Override
    public void updateInputs(DistanceSensorIOInputs inputs) {
        inputs.distance = proxSensor.getValue();
        inputs.sustain = this.sustain;
    }

    @Override
    public void increaseSustain() {
        sustain++;
    }

    @Override
    public void resetSustain() {
        sustain = 0;
    }
}
