package com.peninsula.frc2022.behavior.routines.superstructure;

import com.peninsula.frc2022.behavior.TimeoutRoutineBase;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.ReadOnly;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.subsystems.Indexer;

public class IndexerFeedRoutine extends TimeoutRoutineBase {

  public IndexerFeedRoutine(double timeout) {
    super(timeout);
  }

  @Override
  public void update(@ReadOnly Commands commands, @ReadOnly RobotState state) {
    commands.indexerWanted = Indexer.State.FEED;
  }

  @Override
  public void stop(Commands commands, @ReadOnly RobotState state) {
    commands.indexerWanted = Indexer.State.INDEX;
  }

  @Override
  public boolean checkIfFinishedEarly(@ReadOnly RobotState state) {
    return false;
  }
}
