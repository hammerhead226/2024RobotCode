package frc.robot.statemachines;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class TrapStateMachine {
  private final Elevator elevator;
  private final Pivot pivot;
  private final Shooter shooter;

  public TrapStateMachine(Elevator elevator, Shooter shooter, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
  }

  public enum TRAP_STATES {
    NONE,
    PIVOT,
    EXTEND,
    RETRACT_STOW
  }

  private TRAP_STATES targetState = TRAP_STATES.NONE;

  public TRAP_STATES getTargetState() {
    return targetState;
  }

  public void advanceTargetState() {
    switch (targetState) {
      case NONE:
        targetState = TRAP_STATES.PIVOT;
        break;
      case PIVOT:
        targetState = TRAP_STATES.EXTEND;
        break;
      case EXTEND:
        targetState = TRAP_STATES.RETRACT_STOW;
        break;
      case RETRACT_STOW:
        targetState = TRAP_STATES.NONE;
        break;
      default:
        targetState = TRAP_STATES.NONE;
        break;
    }
    Logger.recordOutput("Trap Target State", targetState);
  }
}
