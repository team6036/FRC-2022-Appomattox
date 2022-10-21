package com.peninsula.frc2022.behavior.routines.superstructure;

import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.IntakeArm;

public class IntakeUpRoutine extends TimeoutRoutineBase {

  public IntakeUpRoutine(double time) {
    super(time);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.intakeArmWanted = IntakeArm.State.UP;
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
