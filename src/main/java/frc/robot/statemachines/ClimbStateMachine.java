package frc.robot.statemachines;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class ClimbStateMachine {
  private final Elevator elevator;
  private final Pivot pivot;
  private final Shooter shooter;

  public ClimbStateMachine(Elevator elevator, Shooter shooter, Pivot pivot) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.shooter = shooter;
  }

  // public enum CLIMB_STATES {
  //   NONE,
  //   PIVOT,
  //   EXTEND,
  //   RETRACT_CLIMB,
  //   EXCHANGE_HOOK,

  //   TRAP_STAGE_1,
  //   TRAP_STAGE_2,
  //   TRAP_STAGE_3,
  //   TRAP_STAGE_4,
  //   SHOOT,
  //   DONE
  // }

  public enum CLIMB_STATES {
    NONE,
    PIVOT_CLIMB,
    RETRACT_CLIMB,
    ENGAGE_STATIC_HOOKS,
    ENGAGE_LOWER_SHOOTER_HOOKS,
    ENGAGE_LOWER_SHOOTER_HOOKS_PART_TWO,
    ALIGN_TO_TRAP,
    SHOOT_NOTE,
    DONE
  }

  private CLIMB_STATES targetState = CLIMB_STATES.NONE;

  public CLIMB_STATES getTargetState() {
    return targetState;
  }

  public void advanceTargetState() {
    // switch (targetState) {
    //   case NONE:
    //     // move angle to climbing pivot angle and extend elevator when done
    //     targetState = CLIMB_STATES.PIVOT;
    //     break;
    //   case PIVOT:
    //     // retract elevator
    //     targetState = CLIMB_STATES.EXTEND;
    //     break;
    //   case EXTEND:
    //     targetState = CLIMB_STATES.RETRACT_CLIMB;
    //     break;
    //   case RETRACT_CLIMB:
    //     targetState = CLIMB_STATES.EXCHANGE_HOOK;
    //     break;
    //   case EXCHANGE_HOOK:
    //     // trap scoring sequence
    //     targetState = CLIMB_STATES.TRAP_STAGE_1;
    //     break;
    //   case TRAP_STAGE_1:
    //     // do nothing lol
    //     targetState = CLIMB_STATES.TRAP_STAGE_2;
    //     break;
    //   case TRAP_STAGE_2:
    //     // do nothing lol
    //     targetState = CLIMB_STATES.TRAP_STAGE_3;
    //     break;
    //   case TRAP_STAGE_3:
    //     // do nothing lol
    //     targetState = CLIMB_STATES.TRAP_STAGE_4;
    //     break;
    //   case TRAP_STAGE_4:
    //     // do nothing lol
    //     targetState = CLIMB_STATES.SHOOT;
    //     break;
    //   case SHOOT:
    //     targetState = CLIMB_STATES.DONE;
    //   case DONE:
    //     break;
    //   default:
    //     targetState = CLIMB_STATES.NONE;
    //     break;
    // }
    switch (targetState) {

        //      ENGAGE_LOWER_SHOOTER_HOOKS,
        // ALIGN_TO_TRAP,
        // SHOOT_NOTE,
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
        targetState = CLIMB_STATES.ENGAGE_STATIC_HOOKS;
        break;
      case ENGAGE_STATIC_HOOKS:
        targetState = CLIMB_STATES.ENGAGE_LOWER_SHOOTER_HOOKS;
        break;
      case ENGAGE_LOWER_SHOOTER_HOOKS:
        // do nothing lol
        targetState = CLIMB_STATES.ENGAGE_LOWER_SHOOTER_HOOKS_PART_TWO;
        break;
      case ENGAGE_LOWER_SHOOTER_HOOKS_PART_TWO:
        // do nothing lol
        targetState = CLIMB_STATES.ALIGN_TO_TRAP;
        break;
      case ALIGN_TO_TRAP:
        targetState = CLIMB_STATES.SHOOT_NOTE;
        break;
      case SHOOT_NOTE:
        targetState = CLIMB_STATES.DONE;
        break;
      case DONE:
        break;
      default:
        targetState = CLIMB_STATES.NONE;
        break;
    }
    Logger.recordOutput("Climb Target State", targetState);
  }
}
