package frc.robot.statemachines;

import java.lang.invoke.MethodHandles.Lookup.ClassOption;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ClimbStateMachine {
    private final Elevator elevator;
    private final Pivot pivot;
    private final Shooter shooter;

    public ClimbStateMachine(Elevator elevator, Shooter shooter, Pivot pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
        this.shooter = shooter;
    }
    public enum CLIMB_STATES {
        NONE,
        EXTEND_CLIMB,
        RETRACT_CLIMB,
        SCORE_TRAP,
    }

    private CLIMB_STATES targetState = CLIMB_STATES.NONE;
    int counter = 0;

    public CLIMB_STATES getTargetState() {
        return targetState;
    }

    public void advanceTargetState() {
        if (atTargetState()) counter = Math.min(counter+1, 3);

        switch (counter) {
            case 0:
                targetState = CLIMB_STATES.NONE;
                break;
            case 1:
                targetState = CLIMB_STATES.EXTEND_CLIMB;
                break;
            case 2:
                targetState = CLIMB_STATES.RETRACT_CLIMB;
                break;
            case 3:
                targetState = CLIMB_STATES.SCORE_TRAP;
            default:
                targetState = CLIMB_STATES.NONE;
                break;
        }
        Logger.recordOutput("Climb Target State", targetState);
    }

    public boolean atTargetState() {
        if (elevator.extenderAtSetpoint() && pivot.pivotAtSetpoint() && shooter.atFlywheelSetpoints()) {
            return true;
        }
        return false;
    }
}
