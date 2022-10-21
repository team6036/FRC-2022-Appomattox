package com.peninsula.frc2022.auto.shooting.preChezy;

import com.pathplanner.lib.PathPlanner;
import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.ParallelRaceRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.*;

public class Citrus2Ball2Rude implements AutoBase {

  @Override
  public RoutineBase getRoutine() {
    var setInitialOdometry = new DriveSetOdometryRoutine(10.4, 3.04, 136.00);

    var goBack =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("MoveBackClimber", 3, 2)),
            new IntakeBallRoutine(10.0));

    var shootIntoHub =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(0.5), new IntakeBallRoutine(0.5), new IndexerFeedRoutine(0.5));

    var goToFarRude =
        new ParallelRaceRoutine(
            new DrivePathRoutine(PathPlanner.loadPath("ClimberBallToRudeFar", 1, 1)),
            new IntakeBallRoutine(10.0),
            new IndexerRoutine(10.0));

    var spit = new ParallelRaceRoutine(new IndexerFeedRoutine(1.0), new ShooterSpitRoutine(1.0));

    var goFarRudeToWallRude =
        new ParallelRaceRoutine(
            new DrivePathRoutine(PathPlanner.loadPath("RudeFarToRudeWall", 2, 1.3)),
            new IntakeBallRoutine(10.0),
            new IndexerRoutine(10.0),
            new ShooterSpitRoutine(10.0));

    var spit2 = new ParallelRaceRoutine(new IndexerFeedRoutine(1.0), new ShooterSpitRoutine(1.0));

    return new SequentialRoutine(
        setInitialOdometry, goBack, shootIntoHub, goToFarRude, spit, goFarRudeToWallRude, spit2);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
