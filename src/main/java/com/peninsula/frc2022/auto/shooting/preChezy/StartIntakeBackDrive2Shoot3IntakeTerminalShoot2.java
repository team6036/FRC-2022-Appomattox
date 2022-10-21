package com.peninsula.frc2022.auto.shooting.preChezy;

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

public class StartIntakeBackDrive2Shoot3IntakeTerminalShoot2 implements AutoBase {

  @Override
  public RoutineBase getRoutine() {

    var setInitialOdometry = new DriveSetOdometryRoutine(8.90, 6.41, -88.00);

    var moveBackAlignedPath =
        new ParallelRaceRoutine(
            new IntakeBallRoutine(10.0),
            new ShooterVisionRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("MoveBackAlign", 2, 1)));

    var shoot2 =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(0.5), new IntakeBallRoutine(0.5), new IndexerFeedRoutine(0.5));

    var ball1To2Path =
        new ParallelRaceRoutine(
            new IntakeBallRoutine(10.0),
            new ShooterVisionRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("Ball1to2Align", 2.7, 1.8)));

    var alignAndShoot1 =
        new ParallelRoutine(
            new IntakeBallRoutine(0.7), new ShooterVisionRoutine(0.7), new IndexerFeedRoutine(0.7));

    var ball2ToTerminalIntake =
        new SequentialRoutine(
            new ParallelRaceRoutine(
                new DrivePathRoutine(PathPlanner.loadPath("Ball2ToTerminal", 4, 2)),
                new IntakeBallRoutine(10.0)));

    var terminalToAlignTarmac =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(10.0),
            new DrivePathRoutine(PathPlanner.loadPath("Ball3Align", 4, 3)),
            new IntakeBallRoutine(10.0));

    var alignAndShootTerminalBalls =
        new ParallelRaceRoutine(
            new ShooterVisionRoutine(1.5),
            new IntakeBallRoutine(2.0),
            new SequentialRoutine(new TimedRoutine(0.5), new IndexerFeedRoutine(1.0)));

    return new SequentialRoutine(
        //				down,
        setInitialOdometry,
        moveBackAlignedPath,
        shoot2,
        ball1To2Path,
        alignAndShoot1,
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
