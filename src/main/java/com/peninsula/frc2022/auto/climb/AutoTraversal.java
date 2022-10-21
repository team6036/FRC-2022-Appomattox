package com.peninsula.frc2022.auto.climb;

import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.*;
import com.peninsula.frc2022.behavior.routines.TimedRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.IntakeOffRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.ShooterIdleRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.climb.*;
import com.peninsula.frc2022.config.ClimberConstants;
import com.peninsula.frc2022.subsystems.controllers.climb.AutoClimbController;

public class AutoTraversal implements AutoBase {
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
            new ParallelRoutine(
                new LeftStretchRoutine(0.2),
                new SequentialRoutine(
                    new TimedRoutine(0.3),
                    new RightExtendPartialRoutine(0.2)))); // Grab high and extend for traversal

    var extendForTraversalRight =
        new SequentialRoutine(
            new AwaitRoutine(
                x ->
                    x.gyroPitch.getDegrees() - x.pitchSet < ClimberConstants.traversalExtendDegrees
                        && atPose(
                            x.rightTick,
                            AutoClimbController.RightArmState.CLOSE_TO_EXTEND.ticks,
                            ClimberConstants.atPoseEpsilon)),
            new RightExtendRoutine(0.2)); // Extend right arm for traversal

    var pullOntoTraversal =
        new SequentialRoutine(
            new AwaitRoutine(
                x ->
                    x.gyroPitch.getDegrees() - x.pitchSet > ClimberConstants.contactTraversalDegrees
                        && atPose(
                            x.rightTick,
                            AutoClimbController.RightArmState.FULL_EXTEND.ticks,
                            ClimberConstants.atPoseEpsilon)),
            new RightDownRoutine(2.0));

    return new ParallelRaceRoutine(
        new SequentialRoutine(
            pullRightAndExtendLeftPartial,
            extendLeftAllWay,
            pullHighLeftAndExtendRightPartial,
            extendForTraversalRight,
            pullOntoTraversal),
        new ShooterIdleRoutine(20.0),
        new IntakeOffRoutine(20.0));
  }

  public boolean atPose(double current, double wanted, double epsilon) {
    return Math.abs(current - wanted) <= epsilon;
  }
}
