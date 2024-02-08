package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ElevatorExtenderIOTalonFX implements ElevatorExtenderIO{
      private final TalonFX falcon;
    private final StatusSignal<Double> elevatorPosition;
    private final StatusSignal<Double> elevatorVelocity;
    private final StatusSignal<Double> appliedVolts;
    private final StatusSignal<Double> currentAmps;

    public ElevatorExtenderIOTalonFX(int id) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackSensorSource =  FeedbackSensorSourceValue.RotorSensor;
        
        falcon = new TalonFX(id);

        falcon.getConfigurator().apply(config);

        elevatorPosition = falcon.getPosition();
        elevatorVelocity = falcon.getVelocity();
        appliedVolts = falcon.getMotorVoltage();
        currentAmps = falcon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, elevatorPosition, elevatorVelocity, appliedVolts, currentAmps);

    }
    
    @Override
    public void updateInputs(ElevatorExtenderIOInputs inputs) {
        inputs.elevatorPosition = Units.rotationsToDegrees(elevatorPosition.getValueAsDouble());
        inputs.elevatorVelocity = Units.rotationsPerMinuteToRadiansPerSecond(elevatorVelocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        
    }

    @Override 
    public void setVelocity(double velocity) {
       falcon.setControl(new VelocityVoltage(velocity));
    }
     
    @Override
    public void setPosition(double position) {
        falcon.setControl(new PositionVoltage(position));
    }
    
    @Override
    public void stop() {
        falcon.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        Slot0Configs config = new Slot0Configs();

        config.kP = kP;
        config.kI = kI;
        config.kD = kD;

        falcon.getConfigurator().apply(config);
    }
}
