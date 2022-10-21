package com.peninsula.frc2022.behavior.routines.superstructure;

import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;

public class ShooterSpitRoutine extends TimeoutRoutineBase {

  public ShooterSpitRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.setShooterRPM(700);
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {
    commands.setShooterIdle();
  }

  @Override
  public boolean checkIfFinishedEarly(RobotState state) {
    return false;
  }
}
