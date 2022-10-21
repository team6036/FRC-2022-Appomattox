package com.peninsula.frc2022.auto.climb;

import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.AwaitRoutine;
import com.peninsula.frc2022.behavior.ParallelRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.climb.*;
import com.peninsula.frc2022.config.ClimberConstants;
import com.peninsula.frc2022.subsystems.controllers.climb.AutoClimbController;

public class AutoHigh implements AutoBase {
  @Override
  public RoutineBase getRoutine() {
    var pullRightAndExtendLeftPartial =
        new ParallelRoutine(
            new RightStretchRoutine(0.2),
            new LeftExtendPartialRoutine(0.2)); // Partial to fit under high bar

    var extendLeftAllWay =
        new SequentialRoutine(
            new AwaitRoutine(
                x ->
                    x.gyroPitch.getDegrees() - x.pitchSet > ClimberConstants.pastHighExtendDegrees
                        && atPose(
                            Math.abs(x.leftTick),
                            AutoClimbController.LeftArmState.CLOSE_TO_EXTEND.ticks,
                            ClimberConstants.atPoseEpsilon)),
            new LeftExtendRoutine(0.2)); // To latch onto high

    var pullHighLeftAndExtendRightPartial =
        new SequentialRoutine(
            new AwaitRoutine(
                x ->
                    x.gyroPitch.getDegrees() - x.pitchSet < ClimberConstants.contactHighDegrees
                        && atPose(
                            x.leftTick,
                            AutoClimbController.LeftArmState.FULL_EXTEND.ticks,
                            ClimberConstants.atPoseEpsilon)),
            new LeftStretchRoutine(0.2)); // Grab high and extend for traversal

    return new SequentialRoutine(
        pullRightAndExtendLeftPartial, extendLeftAllWay, pullHighLeftAndExtendRightPartial);
  }

  public boolean atPose(double current, double wanted, double epsilon) {
    return Math.abs(current - wanted) <= epsilon;
  }
}
