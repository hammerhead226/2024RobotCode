package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.led.LED;

public class BatteryDisplay extends Command {
  private final LED led;

  public BatteryDisplay(LED led) {
    this.led = led;
    addRequirements(led);
  }

  @Override
  public void execute() {
    led.setLEDs(226, 0, 0, 0, 33, 24);
    led.setLEDs(
        0, 226, 0, 0, 33, (int) (24 * normalize(RobotController.getBatteryVoltage(), 11.0, 12.5)));
    led.setLEDs(255, 255, 255, 0, (int) Constants.LEDConstants.debugLEDIdx.get(), 1);
  }

  private double normalize(double x, double min, double max) {
    double result = (x - min) / (max - min);
    if (result > 1) {
      return 1;
    }
    if (result < 0) {
      return 0;
    }
    return result;
  }
}
