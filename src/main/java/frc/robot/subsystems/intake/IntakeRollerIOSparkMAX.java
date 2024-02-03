package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeRollerIOSparkMAX implements IntakeRollerIO{
    private final CANSparkMax rollers;
    private final SparkPIDController pid;

    public IntakeRollerIOSparkMAX(int id) {
        rollers = new CANSparkMax(id, MotorType.kBrushless);
        rollers.restoreFactoryDefaults();
        rollers.setIdleMode(IdleMode.kCoast);
        
        rollers.setSmartCurrentLimit(30);
        rollers.setCANTimeout(250);
        rollers.burnFlash();

        pid = rollers.getPIDController();
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.rollerVelocity = rollers.getEncoder().getVelocity();

        inputs.appliedVolts = rollers.getBusVoltage();
        inputs.currentAmps = rollers.getOutputCurrent();
    }

    @Override
    public void setVelocity(double velocity) {
        pid.setReference(velocity, ControlType.kVelocity);
    }

    @Override
    public void stop() {
        rollers.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setP(kP, 0);
        pid.setI(kI, 0);
        pid.setD(kD, 0);
    }
}
