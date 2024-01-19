// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;

public class LED_IOSpark implements LED_IO {
  LED_STATE ledState;
  Spark led;

  public LED_IOSpark(int channel) {
    led = new Spark(channel);
  }

  @Override
  public void updateInputs(LED_IOInputs inputs) {
    inputs.ledState = ledState;
  }

  @Override
  public void noBumpersPressed() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      ledState = LED_STATE.BLUE;
      led.set(Constants.COLOR_BLUE);
    } else {
      ledState = LED_STATE.RED;
      led.set(Constants.COLOR_RED);
    }
  }

  @Override
  public void setColor(LED_STATE state) {
    ledState = state;
    switch (ledState) {
      case RED:
        led.set(Constants.COLOR_RED);
        break;
      case BLUE:
        led.set(Constants.COLOR_BLUE);
        break;
      case YELLOW:
        led.set(Constants.COLOR_YELLOW);
        break;
      case VIOLET:
        led.set(Constants.COLOR_VIOLET);
        break;
      case OFF:
        led.close();
        break;
    }
  }
}
