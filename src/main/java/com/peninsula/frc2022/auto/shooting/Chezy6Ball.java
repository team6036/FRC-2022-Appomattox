package com.peninsula.frc2022.auto.shooting;

import com.pathplanner.lib.PathPlanner;
import com.peninsula.frc2022.auto.AutoBase;
import com.peninsula.frc2022.behavior.ParallelRaceRoutine;
import com.peninsula.frc2022.behavior.ParallelRoutine;
import com.peninsula.frc2022.behavior.RoutineBase;
import com.peninsula.frc2022.behavior.SequentialRoutine;
import com.peninsula.frc2022.behavior.routines.TimedRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DrivePathRoutine;
import com.peninsula.frc2022.behavior.routines.drive.DriveSetOdometryRoutine;
import com.peninsula.frc2022.behavior.routines.superstructure.*;

public class Chezy6Ball implements AutoBase {

  @Override
  public RoutineBase getRoutine() {

    var setInitialOdometry = new DriveSetOdometryRoutine(8.90, 6.41, -88.00);

    var moveBackAlignedPath =
        new ParallelRaceRoutine(
            new IntakeBallRoutine(0.8),
            new ShooterOdometryRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("MoveBackAlign", 2.1, 2)));

    var shoot3 =
        new ParallelRoutine(
            new ShooterOdometryRoutine(0.8),
            new IndexerFeedRoutine(0.8),
            new IntakeBallRoutine(0.8));

    var ball1To2Path =
        new ParallelRaceRoutine(
            new IntakeBallRoutine(10.0),
            new ShooterOdometryRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("Ball1to2Align", 1.7, 1.8)));

    var shootSecond2 =
        new ParallelRoutine(
            new IntakeBallRoutine(0.7),
            new ShooterOdometryRoutine(0.7),
            new IndexerFeedRoutine(0.7));

    var ball2ToTerminalIntake =
        new SequentialRoutine(
            new ParallelRaceRoutine(
                new DrivePathRoutine(PathPlanner.loadPath("Ball2ToTerminal", 4, 2)),
                new IntakeBallRoutine(10.0)));

    var terminalToAlignTarmac =
        new ParallelRaceRoutine(
            new ShooterOdometryRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("Ball3Align", 3, 3)),
            new IntakeBallRoutine(10.0));

    var alignAndShootTerminalBalls =
        new ParallelRaceRoutine(
            new ShooterOdometryRoutine(2.0),
            new IntakeBallRoutine(2.0),
            new SequentialRoutine(new TimedRoutine(0.5), new IndexerFeedRoutine(1.5)));

    return new SequentialRoutine(
        setInitialOdometry,
        moveBackAlignedPath,
        shoot3,
        ball1To2Path,
        shootSecond2,
        ball2ToTerminalIntake,
        new ParallelRoutine(new TimedRoutine(0.5), new IntakeBallRoutine(0.5)),
        terminalToAlignTarmac,
        alignAndShootTerminalBalls);
  }

  @Override
  public String getName() {
    return AutoBase.super.getName();
  }
}
