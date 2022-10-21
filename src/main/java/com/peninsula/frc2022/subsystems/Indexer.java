package com.peninsula.frc2022.subsystems;

import com.peninsula.frc2022.config.IndexerConstants;
import com.peninsula.frc2022.robot.Commands;
import com.peninsula.frc2022.robot.RobotState;
import com.peninsula.frc2022.util.control.ControllerOutput;

public class Indexer extends SubsystemBase {

  public enum State {
    INDEX,
    FEED
  }

  public enum ObservedState {
    NONE,
    PEEK,
    INDEXER_KICKER,
    KICKER
  }

  private static Indexer sInstance = new Indexer();
  protected ControllerOutput mIntakeIndexerOutput = new ControllerOutput(),
      mKickerOutput = new ControllerOutput();

  private Indexer() {}

  public static Indexer getInstance() {
    return sInstance;
  }

  public ControllerOutput getIntakeIndexerOutput() {
    return mIntakeIndexerOutput;
  }

  public ControllerOutput getKickerOutput() {
    return mKickerOutput;
  }

  @Override
  public void update(Commands commands, RobotState state) {

    ObservedState first = ObservedState.NONE;
    ObservedState second =
        ObservedState.NONE; // should only ever be in the peek or none state hopefully

    ObservedState[] st = observeState(first, second, state);

    first = st[0];
    second = st[1];

    if (first == ObservedState.NONE || first == ObservedState.PEEK) {
      mIntakeIndexerOutput.setPercentOutput(IndexerConstants.intakeIndexerPo);
    } else {
      mIntakeIndexerOutput.setIdle();
      System.out.println("IN");
    }

    if (commands.indexerWanted == State.FEED) {
      mIntakeIndexerOutput.setPercentOutput(IndexerConstants.intakeIndexerPo);
      mKickerOutput.setPercentOutput(0.4);
      System.out.println("FEED");
    } else {
      mKickerOutput.setIdle();
    }
  }

  public ObservedState[] observeState(ObservedState first, ObservedState second, RobotState state) {
    if (state.lastIRBlocked) {
      first = ObservedState.KICKER;
      if (state.firstIRBlocked) {
        second = ObservedState.PEEK;
      }
    } else { // First ball not at kicker
      if (state.firstIRBlocked) {
        first = ObservedState.PEEK;
        if (state.secondIRBlocked) {
          first = ObservedState.INDEXER_KICKER;
        }
      }
    }
    return new ObservedState[] {first, second};
  }
}
