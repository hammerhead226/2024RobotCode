// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;

public class LED_IOCANdle implements LED_IO {
  LED_STATE ledState;

  CANdle candle;
  StrobeAnimation flashGreen = new StrobeAnimation(0, 204, 0, 0, 0.01, 56);
  StrobeAnimation flashRed = new StrobeAnimation(179, 30, 0, 0, 0.01, 56);
  StrobeAnimation fashYellow = new StrobeAnimation(255, 255, 0, 0, 0.01, 56);

  ColorFlowAnimation off = new ColorFlowAnimation(0, 0, 0, 0, 0.01, 0, Direction.Forward, 28);
  ColorFlowAnimation wayBlue =
      new ColorFlowAnimation(0, 0, 240, 0, 0.01, 24, Direction.Forward, 32);
  ColorFlowAnimation wayYellow =
      new ColorFlowAnimation(255, 255, 0, 0, 0, 56, Direction.Forward, 0);
  ColorFlowAnimation wayRed = new ColorFlowAnimation(240, 0, 0, 0, 0.01, 28, Direction.Forward, 28);
  ColorFlowAnimation wayGreen =
      new ColorFlowAnimation(0, 240, 0, 0, 0.01, 28, Direction.Forward, 28);

  public LED_IOCANdle(int channel, String CANBUS) {
    // led = new Spark(channel);
    candle = new CANdle(channel, CANBUS);
    ledState = Constants.LED_STATE.RED;

    CANdleConfiguration configs = new CANdleConfiguration();
    // CANdleControlFrame.CANdle_Control_1_General(0x4000);
    configs.stripType = LEDStripType.RGB;
    configs.brightnessScalar = 0.7;

    candle.configAllSettings(configs);

    setColor(ledState);
  }

  @Override
  public void updateInputs(LED_IOInputs inputs) {
    inputs.ledState = ledState;
  }

  @Override
  public void noBumpersPressed() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setColor(LED_STATE.BLUE);
      // led.set(Constants.LEDConstants.COLOR_BLUE);

    } else {
      setColor(LED_STATE.RED);
      // led.set(Constants.LEDConstants.COLOR_RED);
    }
  }

  @Override
  public void setColor(LED_STATE state) {
    ledState = state;
    // candle.setLEDs(0, 0, 0);
    switch (ledState) {
      case RED:
        candle.setLEDs(255, 0, 0, 0, 28, 28);
        break;
      case BLUE:
        // led.set(Constants.LEDConstants.COLOR_BLUE);
        candle.setLEDs(0, 0, 255, 0, 32, 24);
        break;
      case YELLOW:
        // led.set(Constants.LEDConstants.COLOR_YELLOW);
        candle.setLEDs(255, 255, 0, 0, 28, 28);
        break;
      case VIOLET:
        // led.set(Constants.LEDConstants.COLOR_VIOLET);
        break;
      case GREEN:
        candle.setLEDs(0, 255, 0, 0, 28, 28);
        break;
      case FLASHING_GREEN:
        candle.animate(flashGreen, 0);
        break;
      case FLASHING_RED:
        candle.animate(flashRed, 0);
        break;
      case OFF:
        // candle.animate(off);
        candle.setLEDs(0, 0, 0, 0, 0, 0);
        // candle.t
        break;
    }
  }
}
