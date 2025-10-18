// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;

public class LED_IOCANdle implements LED_IO {
  LED_STATE ledState;

  CANdle candle;
  StrobeAnimation flashGreen = new StrobeAnimation(0, 204, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashRed = new StrobeAnimation(204, 0, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashBlue = new StrobeAnimation(0, 0, 255, 0, 0.01, 57 + 24);
  StrobeAnimation fashYellow = new StrobeAnimation(255, 255, 0, 0, 0.01, 57 + 24);
  StrobeAnimation flashWhite = new StrobeAnimation(255, 255, 255, 255, 0.01, 57 + 24);
  StrobeAnimation halfFlashWhite = new StrobeAnimation(255, 255, 255, 255, 0.01, 28 + 24, 0);

  FireAnimation rainbow = new FireAnimation(0.3, 0.03, 57 + 24, 0.1, 0.1);
  // ColorFlowAnimation rainbow = new ColorFlowAnimation(0, 0, 255, 0, 0.343, 57,
  // Direction.Forward);

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
    ledState = Constants.LED_STATE.BLUE;

    CANdleConfiguration configs = new CANdleConfiguration();
    // CANdleControlFrame.CANdle_Control_1_General(0x4000);
    configs.stripType = LEDStripType.RGB;
    configs.brightnessScalar = 0.8;

    candle.configAllSettings(configs);
    // setColor(LED_STATE.OFF);

    setLEDState(ledState);
  }

  @Override
  public void updateInputs(LED_IOInputs inputs) {
    inputs.ledState = ledState;
    // inputs.currentAmps = candle.getCurrent();
  }

  @Override
  public void noBumpersPressed() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      setLEDState(LED_STATE.BLUE);
      // led.set(Constants.LEDConstants.COLOR_BLUE);

    } else {
      setLEDState(LED_STATE.RED);
      // led.set(Constants.LEDConstants.COLOR_RED);
    }
  }

  @Override
  public void setLEDState(LED_STATE state) {
    ledState = state;
    // candle.setLEDs(0, 0, 0);
    switch (ledState) {
      case RED:
        // TODO:: MANUAL INTAKE | RED
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 0, 0, 0, 57 + 24);
        break;
      case BLUE:
        // TODO:: DEFAULT COLOR | BLUE
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 255, 0, 0, 57 + 24);
        break;
      case YELLOW:
        candle.clearAnimation(0);
        // led.set(Constants.LEDConstants.COLOR_YELLOW);
        // candle.setLEDs(255, 255, 0, 0, 32, 25);
        candle.setLEDs(255, 255, 0, 0, 0, 57 + 24);
        break;
      case VIOLET:
        // led.set(Constants.LEDConstants.COLOR_VIOLET);
        break;
      case GREY:
        // TODO:: SHOOT NOTE | GREY
        candle.clearAnimation(0);
        candle.setLEDs(137, 129, 123);
        break;
      case GREEN:
        // TODO:: SENSOR SEES NOTE | GREEN
        candle.clearAnimation(0);
        candle.setLEDs(0, 255, 0, 0, 0, 57 + 24);
        break;

      case PURPLE:
        candle.clearAnimation(0);
        candle.setLEDs(255, 0, 255);
        break;
      case PAPAYA_ORANGE:
        candle.clearAnimation(0);
        candle.setLEDs(255, 30, 0);
        break;
      case WILLIAMS_BLUE:
        candle.clearAnimation(0);
        candle.setLEDs(0, 160, 222);
        break;
      case HALF_FLASH_RED_HALF_FLASH_WHITE:
        // TODO:: INTAKING FROM SOURCE | HALF_FLASH_RED_HALF_FLASH_WHITE
        candle.clearAnimation(0);
        candle.animate(halfFlashWhite);
        // candle.setLEDs(255, 0, 0, 0, 32, 28);
        candle.setLEDs(255, 0, 0, 0, 32, 25);
        break;
      case FLASHING_WHITE:
        // TODO:: SIGNAL TO HUMAN PLAYER | FLASHING WHITE
        candle.animate(flashWhite, 0);
        break;
      case FLASHING_GREEN:
        // TODO:: AIMBOT | FLASHING GREEN
        candle.animate(flashGreen, 0);
        break;
      case FLASHING_RED:
        // TODO:: TRAJECTORY INTAKE | FLASHING RED
        candle.animate(flashRed, 0);
        break;
      case FLASHING_BLUE:
        candle.animate(flashBlue, 0);
        break;
      case FIRE:
        // TODO:: DISABLED | FIRE
        candle.animate(rainbow, 0);
        break;
      case OFF:
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0, 0, 0, 57 + 24);
        break;
      default:
        break;
    }
  }

  @Override
  public void setLEDs(int r, int g, int b, int w, int startIdx, int count) {
    candle.setLEDs(r, g, b, w, startIdx, count);
  }
}
