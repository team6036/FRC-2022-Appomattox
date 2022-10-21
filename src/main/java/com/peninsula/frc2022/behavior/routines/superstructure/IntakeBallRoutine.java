package com.peninsula.frc2022.behavior.routines.superstructure;

import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Intake;

public class IntakeBallRoutine extends TimeoutRoutineBase {

  public IntakeBallRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.intakeWanted = Intake.State.RUN;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {
    commands.intakeWanted = Intake.State.OFF;
  }

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    return false;
  }
}
