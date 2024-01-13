package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorPivotIO.ElevatorPivotIOInputs;

public class Elevator extends SubsystemBase {
    private final ElevatorPivotIO pivot;
    private final ElevatorPivotIOInputs pInputs = new ElevatorPivotIOInputs();
}
