package com.peninsula.frc2022.auto.shooting;

import com.pathplanner.lib.PathPlanner;
import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.ParallelRaceRoutine;
import com.peninsula.frc2022.behavior.ParallelRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.*;

public class Chezy3Ball2Rude implements AutoBase {
  @Override
  public RoutineBase getRoutine() {

    var setInitialOdometry = new DriveSetOdometryRoutine(10.4, 3.04, 136.00);

    var goBack =
        new ParallelRaceRoutine(
            new ShooterOdometryRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("MoveBackClimber", 1.5, 1.5)),
            new IntakeBallRoutine(10.0),
            new IntakeDownRoutine(10.0));

    var shoot3 =
        new ParallelRoutine(
            new IntakeBallRoutine(0.7),
            new IndexerFeedRoutine(0.7),
            new ShooterOdometryRoutine(0.7));

    var goToFarRude =
        new ParallelRaceRoutine(
            new DrivePathRoutine(PathPlanner.loadPath("ClimberBallToRudeFar", 1.3, 1)),
            new IntakeBallRoutine(10.0),
            new IndexerRoutine(10.0));

    var goFarRudeToWallRude =
        new ParallelRaceRoutine(
            new DrivePathRoutine(PathPlanner.loadPath("RudeFarToRudeWall", 1.3, 1)),
            new IntakeBallRoutine(10.0),
            new IndexerRoutine(10.0),
            new SequentialRoutine(new IntakeUpRoutine(1.5), new IntakeDownRoutine(10.0)));

    var spitRude = new ParallelRoutine(new ShooterSpitRoutine(2.0), new IndexerFeedRoutine(2.0));

    return new SequentialRoutine(
        setInitialOdometry,
        goBack,
        shoot3,
        goToFarRude,
        goFarRudeToWallRude,
        spitRude,
        new IntakeUpRoutine(1.0));
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
