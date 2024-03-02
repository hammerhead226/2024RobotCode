// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;

public class LED_IOCANdle implements LED_IO {
  LED_STATE ledState;
  
  CANdle candle;

  StrobeAnimation flashGreen = new StrobeAnimation(0, 204, 0, 0, 5 ,300);
  StrobeAnimation flashRed = new StrobeAnimation(179, 30, 0, 0, 1, 300);
  ColorFlowAnimation wayBlue = new ColorFlowAnimation(0, 0, 240, 0, 2, 300, Direction.Forward);

  public LED_IOCANdle(int channel, String CANBUS) {
    //led = new Spark(channel);
    candle = new CANdle(channel, CANBUS);
    ledState = Constants.LED_STATE.BLUE;

    CANdleConfiguration configs = new CANdleConfiguration();
    //CANdleControlFrame.CANdle_Control_1_General(0x4000);
    configs.stripType = LEDStripType.RGB;
    configs.brightnessScalar = 0.7;
    
    
    candle.configAllSettings(configs);
    

  }

  @Override
  public void updateInputs(LED_IOInputs inputs) {
    inputs.ledState = ledState;
  }

  @Override
  public void noBumpersPressed() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      ledState = LED_STATE.BLUE;
      //led.set(Constants.LEDConstants.COLOR_BLUE);
      
    } else {
      ledState = LED_STATE.RED;
      //led.set(Constants.LEDConstants.COLOR_RED);
    }
  }

  @Override
  public void setColor(LED_STATE state) {
    ledState = state;
    switch (ledState) {
      case RED:
       // led.set(Constants.LEDConstants.COLOR_RED);
        break;
      case BLUE:
       // led.set(Constants.LEDConstants.COLOR_BLUE);
       candle.animate(wayBlue);
        break;
      case YELLOW:
       // led.set(Constants.LEDConstants.COLOR_YELLOW);
        break;
      case VIOLET:
       // led.set(Constants.LEDConstants.COLOR_VIOLET);
        break;
        case FLASHING_GREEN:
          candle.animate(flashGreen);
        break;
        case FLASHING_RED:
          candle.animate(flashRed);
        break;
      case OFF:
       // led.close();
        break;
    }
  }
}
