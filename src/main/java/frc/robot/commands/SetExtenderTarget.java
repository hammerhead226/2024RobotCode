// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorExtenderIOSim;
import frc.robot.subsystems.Elevator.ElevatorExtenderIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorPivotIOSim;
import frc.robot.subsystems.Elevator.ElevatorPivotIOTalonFX;

public class SetExtenderTarget extends Command {
  /** Creates a new ExtendElevator. */
  private final Elevator elevator;

  private double setPoint;
  
  public SetExtenderTarget(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(elevator);
    setPoint = this.setPoint;

    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorPivotIOTalonFX(RobotMap.ElevatorIDs.PIVOT, RobotMap.ElevatorIDs.CANCODER),new ElevatorExtenderIOTalonFX(RobotMap.ElevatorIDs.EXTENDERS[0], RobotMap.ElevatorIDs.EXTENDERS[1]));
        break;
      case REPLAY:
        elevator = new Elevator(new ElevatorPivotIOSim(), new ElevatorExtenderIOSim());
        break;
      case SIM:
        elevator = new Elevator(new ElevatorPivotIOSim(), new ElevatorExtenderIOSim());
        break;
      default:
        elevator = new Elevator(new ElevatorPivotIOTalonFX(RobotMap.ElevatorIDs.PIVOT, RobotMap.ElevatorIDs.CANCODER),new ElevatorExtenderIOTalonFX(RobotMap.ElevatorIDs.EXTENDERS[0], RobotMap.ElevatorIDs.EXTENDERS[1]));
        break;
    }

  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setExtenderGoal(setPoint);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.extenderAtSetpoint();
  }
}
