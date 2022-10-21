package com.peninsula.frc2022.robot;

import com.peninsula.frc2022.subsystems.*;
import com.peninsula.frc2022.subsystems.controllers.climb.AutoClimbController;

/** Used to produce {@link Commands}'s from human input. Should only be used in robot package. */
public class Control {

  public static final double kDeadBand = 0.05;
  public static final int kOnesTimesZoomAlignButton = 3, kTwoTimesZoomAlignButton = 4;

  /** Modifies commands based on operator input devices. */
  void updateCommands(Commands commands, @ReadOnly RobotState state) {
    updateDriveCommands(commands, state);
    updateSuperstructureCommands(commands, state);
    updateClimberCommands(commands, state);
    updateLightingCommands(commands, state);
    Robot.sLoopDebugger.addPoint("updateCommands");
  }

  private void updateDriveCommands(Commands commands, RobotState state) {
    if (state.driverRbPressed) {
      commands.setShooterDriveBy();
    } else {
      commands.swerveWanted = Swerve.State.TELEOP;
    }
    commands.boostWanted = (state.driverRt > 0.5);
    commands.robotCentricWanted = state.driverLtPressed;
  }

  private void updateSuperstructureCommands(Commands commands, RobotState state) {

    if (state.driverYPressed) {
      //      state.driveOdometry.resetPosition(
      //          new Pose2d(8.90, 6.41, state.gyroHeading), state.gyroHeading);
      state.pitchSet = state.gyroPitch.getDegrees();
    }
    if (state.operatorBPressed) {
      //    if (state.driverRt > 0.2) {
      commands.intakeWanted = Intake.State.RUN;
      commands.intakeArmWanted = IntakeArm.State.DOWN;
    } else {
      commands.intakeArmWanted = IntakeArm.State.UP;
      commands.intakeWanted = Intake.State.OFF;
    }

    if (state.operatorAPressed) {
      commands.indexerWanted = Indexer.State.FEED;
    } else {
      commands.indexerWanted = Indexer.State.INDEX;
    }
    commands.angleWanted += state.driverRightX * 7;
    commands.angleWanted = (commands.angleWanted + 360) % 360;
  }

  private void updateClimberCommands(Commands commands, RobotState state) {

    boolean startClimbSet = state.operatorRtPressed && state.operatorLtPressed;
    boolean startClimbStart = state.operatorRbPressed && state.operatorLbPressed;

    if (startClimbSet) {
      //    if (state.operatorRtPressed) {
      //      commands.leftStateClimbWanted = AutoClimbController.LeftArmState.FULL_EXTEND;
      commands.rightStateClimbWanted = AutoClimbController.RightArmState.FULL_EXTEND;
      commands.zeroRoll = true;
    }

    if (startClimbStart) {
      //    if (state.operatorLtPressed) {
      //      commands.addWantedRoutines(new LeftStretchRoutine(2.0));
      commands.addWantedRoutines(Robot.climbChooser.getSelected());
    }
  }

  private void updateLightingCommands(Commands commands, @ReadOnly RobotState state) {
    commands.lightingWantedState = Lighting.State.IDLE;

    if (commands.swerveWanted == Swerve.State.SHOOT_ON_MOVE) {
      commands.lightingWantedState = Lighting.State.ALIGN;
      if (Math.hypot(
              state.fieldRelativeSpeeds.vyMetersPerSecond,
              state.fieldRelativeSpeeds.vxMetersPerSecond)
          > 0.3) {
        commands.lightingWantedState = Lighting.State.SHOOTING_ON_MOVE_ALIGN;
      }
    }

    if (commands.indexerWanted == Indexer.State.FEED) {
      commands.lightingWantedState = Lighting.State.SHOOTING;
    }

    if (state.gamePeriod == RobotState.GamePeriod.AUTO
        || state.gamePeriod == RobotState.GamePeriod.DISABLED) {
      commands.lightingWantedState = Lighting.State.AUTO;
    }
  }

  public void reset(Commands commands) {
    commands.routinesWanted.clear();
    commands.swerveWanted = Swerve.State.TELEOP;
    commands.boostWanted = false;
    commands.intakeWanted = Intake.State.OFF;
    //    commands.indexerWanted = Indexer.State.OFF;
    commands.visionWanted = Vision.State.ON;
    commands.lightingWantedState = Lighting.State.AUTO;
    commands.setShooterIdle();
    commands.intakeArmWanted = IntakeArm.State.DOWN;
  }
}
