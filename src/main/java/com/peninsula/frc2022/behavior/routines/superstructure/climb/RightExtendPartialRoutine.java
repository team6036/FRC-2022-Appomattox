package com.peninsula.frc2022.behavior.routines.superstructure.climb;

import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Climber;
import com.peninsula.frc2022.subsystems.controllers.climb.AutoClimbController;

public class RightExtendPartialRoutine extends TimeoutRoutineBase {

  public RightExtendPartialRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.climbWantedState = Climber.ClimbState.AUTO;
    commands.rightStateClimbWanted = AutoClimbController.RightArmState.CLOSE_TO_EXTEND;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {}
}
