package com.peninsula.frc2022.behavior;

import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.SubsystemBase;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Predicate;

/** Waits until a condition is met Good to use in sequential routine */
public class AwaitRoutine extends RoutineBase {

  protected Predicate<RobotState> mPredicate;

  public AwaitRoutine(Predicate<RobotState> predicate) {
    mPredicate = predicate;
  }

  @Override
  protected void update(Commands commands, @ReadOnly RobotState state) {}

  @Override
  protected void stop(Commands commands, @ReadOnly RobotState state) {}

  @Override
  public boolean checkFinished(@ReadOnly RobotState state) {

    return mPredicate.test(state);
  }

  public Set<SubsystemBase> getRequiredSubsystems() {
    return new HashSet<>();
  }
}
