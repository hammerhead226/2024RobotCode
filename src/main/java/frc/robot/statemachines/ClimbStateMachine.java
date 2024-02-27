package frc.robot.statemachines;

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
        PIVOT_CLIMB,
        RETRACT_CLIMB,
        SCORE_TRAP,
        DONE
    }

    private CLIMB_STATES targetState = CLIMB_STATES.NONE;

    public CLIMB_STATES getTargetState() {
        return targetState;
    }

    public void advanceTargetState() {
        switch (targetState) {
            case NONE:
                // move angle to climbing pivot angle and extend elevator when done
                targetState = CLIMB_STATES.PIVOT_CLIMB;
                break;
            case PIVOT_CLIMB:
                // retract elevator
                targetState = CLIMB_STATES.RETRACT_CLIMB;
                break;
            case RETRACT_CLIMB:
                // trap scoring sequence
                targetState = CLIMB_STATES.SCORE_TRAP;
            case SCORE_TRAP:
                // do nothing lol
                targetState = CLIMB_STATES.DONE;
            default:
                targetState = CLIMB_STATES.NONE;
                break;
        }
        Logger.recordOutput("Climb Target State", targetState);
        }
}
