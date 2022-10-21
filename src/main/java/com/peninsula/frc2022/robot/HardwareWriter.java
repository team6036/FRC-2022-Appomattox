package com.peninsula.frc2022.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.peninsula.frc2022.config.ClimberConstants;
import com.peninsula.frc2022.config.IndexerConstants;
import com.peninsula.frc2022.config.IntakeConstants;
import com.peninsula.frc2022.config.ShooterConstants;
import com.peninsula.frc2022.subsystems.*;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Set;
import org.photonvision.common.hardware.VisionLEDMode;

public class HardwareWriter {

  public static final int kTimeoutMs = 150, kPidIndex = 0;

  void configureHardware(Set<SubsystemBase> enabledSubsystems) {
    if (enabledSubsystems.contains(Swerve.getInstance())) configureSwerveHardware();
    if (enabledSubsystems.contains(Intake.getInstance())) configureIntakeHardware();
    if (enabledSubsystems.contains(Indexer.getInstance())) configureIndexerHardware();
    if (enabledSubsystems.contains(Shooter.getInstance())) configureShooterHardware();
    if (enabledSubsystems.contains(Climber.getInstance())) configureClimberHardware();
    if (enabledSubsystems.contains(Vision.getInstance())) configureVisionHardware();
    if (enabledSubsystems.contains(Lighting.getInstance())) configureLightingHardware();
  }

  private void configureSwerveHardware() {}

  private void configureIntakeHardware() {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();
    hardware.motor.setNeutralMode(NeutralMode.Brake);
    hardware.motor.setInverted(false);

    hardware.armMotorLeft.follow(hardware.armMotorRight);
    hardware.armMotorRight.setInverted(false);
    hardware.armMotorLeft.setInverted(true);

    hardware.armMotorRight.config_kP(0, IntakeConstants.intakeArmGainsHard.p);
    hardware.armMotorRight.config_kI(0, IntakeConstants.intakeArmGainsHard.i);
    hardware.armMotorRight.config_kD(0, IntakeConstants.intakeArmGainsHard.d);
    hardware.armMotorRight.config_kF(0, IntakeConstants.intakeArmGainsHard.f);

    hardware.armMotorLeft.config_kP(0, IntakeConstants.intakeArmGainsLight.p);
    hardware.armMotorLeft.config_kI(0, IntakeConstants.intakeArmGainsLight.i);
    hardware.armMotorLeft.config_kD(0, IntakeConstants.intakeArmGainsLight.d);
    hardware.armMotorLeft.config_kF(0, IntakeConstants.intakeArmGainsLight.f);

    hardware.armMotorRight.setNeutralMode(NeutralMode.Brake);
    hardware.armMotorLeft.setNeutralMode(NeutralMode.Brake);

    hardware.armMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 1000);
    hardware.armMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 1000);
    hardware.armMotorRight.setSelectedSensorPosition(0.0);
    hardware.armMotorLeft.setSelectedSensorPosition(0.0);
  }

  private void configureIndexerHardware() {
    var hardware = HardwareAdapter.IndexerHardware.getInstance();
    hardware.intakeIndexerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    hardware.intakeIndexerMotor.setInverted(true);
    hardware.kickerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 1000);

    hardware.kickerMotor.config_kP(0, IndexerConstants.kickerPosGains.p, 100);
    hardware.kickerMotor.config_kI(0, IndexerConstants.kickerPosGains.i, 100);
    hardware.kickerMotor.config_kD(0, IndexerConstants.kickerPosGains.d, 100);
    hardware.kickerMotor.config_kF(0, IndexerConstants.kickerPosGains.f, 100);

    hardware.intakeIndexerMotor.getController().setP(IndexerConstants.intakeIndexerGains.p, 0);
    hardware.intakeIndexerMotor.getController().setI(IndexerConstants.intakeIndexerGains.i, 0);
    hardware.intakeIndexerMotor.getController().setD(IndexerConstants.intakeIndexerGains.d, 0);
    hardware
        .intakeIndexerMotor
        .getController()
        .setFF(IndexerConstants.intakeIndexerGains.f, 0); // TODO: sketchy
  }

  private void configureShooterHardware() {
    var hardware = HardwareAdapter.ShooterHardware.getInstance();
    hardware.shooterMasterMotor.configFactoryDefault();
    hardware.shooterSlaveMotor.configFactoryDefault();
    hardware.hoodMotor.configFactoryDefault();
    hardware.shooterMasterMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
    hardware.shooterSlaveMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
    hardware.hoodMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
    hardware.shooterSlaveMotor.setInverted(true);
    hardware.shooterMasterMotor.setInverted(false);
    hardware.hoodMotor.setInverted(true);

    hardware.shooterMasterMotor.setNeutralMode(NeutralMode.Coast);
    hardware.shooterSlaveMotor.setNeutralMode(NeutralMode.Coast);
    hardware.hoodMotor.setNeutralMode(NeutralMode.Coast);

    hardware.shooterSlaveMotor.follow(hardware.shooterMasterMotor);

    hardware.shooterMasterMotor.config_kP(0, ShooterConstants.flywheelGains.p, 1000);
    hardware.shooterMasterMotor.config_kI(0, ShooterConstants.flywheelGains.i, 1000);
    hardware.shooterMasterMotor.config_kD(0, ShooterConstants.flywheelGains.d, 1000);
    hardware.shooterMasterMotor.config_kF(0, ShooterConstants.flywheelGains.f, 1000);

    hardware.hoodMotor.config_kP(0, ShooterConstants.hoodwheelGains.p, 1000);
    hardware.hoodMotor.config_kI(0, ShooterConstants.hoodwheelGains.i, 1000);
    hardware.hoodMotor.config_kD(0, ShooterConstants.hoodwheelGains.d, 1000);
    hardware.hoodMotor.config_kF(0, ShooterConstants.hoodwheelGains.f, 1000);
  }

  private void configureClimberHardware() {
    var hardware = HardwareAdapter.ClimberHardware.getInstance();

    // Left
    hardware.leftPuller.setNeutralMode(NeutralMode.Brake);

    hardware.leftPuller.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    hardware.leftPuller.setSelectedSensorPosition(0.0);

    hardware.leftPuller.configMotionAcceleration(
        ClimberConstants.accelerationTicksPer100MsPerSec, 100);
    hardware.leftPuller.configMotionCruiseVelocity(ClimberConstants.velocityTicksPer100ms, 100);

    hardware.leftPuller.config_kP(0, ClimberConstants.pullerGains.p);
    hardware.leftPuller.config_kI(0, ClimberConstants.pullerGains.i);
    hardware.leftPuller.config_kD(0, ClimberConstants.pullerGains.d);
    hardware.leftPuller.config_kF(0, ClimberConstants.pullerGains.f);

    hardware.leftPuller.configVoltageCompSaturation(12.0, 100);
    hardware.leftPuller.enableVoltageCompensation(true);

    hardware.leftPuller.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, 70, 60, .2));

    hardware.leftPuller.setInverted(true);

    hardware.rightPuller.setNeutralMode(NeutralMode.Brake);

    hardware.rightPuller.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    hardware.rightPuller.setSelectedSensorPosition(0.0);
    hardware.rightPuller.setInverted(true);

    hardware.rightPuller.configMotionAcceleration(
        ClimberConstants.accelerationTicksPer100MsPerSec, 100);
    hardware.rightPuller.configMotionCruiseVelocity(ClimberConstants.velocityTicksPer100ms, 100);

    hardware.rightPuller.config_kP(0, ClimberConstants.pullerGains.p);
    hardware.rightPuller.config_kI(0, ClimberConstants.pullerGains.i);
    hardware.rightPuller.config_kD(0, ClimberConstants.pullerGains.d);
    hardware.rightPuller.config_kF(0, ClimberConstants.pullerGains.f);

    hardware.rightPuller.configVoltageCompSaturation(12.0, 100);
    hardware.rightPuller.enableVoltageCompensation(true);

    hardware.rightPuller.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, 60, 60, .2));
  }

  private void configureVisionHardware() {
    var hardware = HardwareAdapter.VisionHardware.getInstance();
    hardware.camera.setPipelineIndex(0);
    hardware.camera.setLED(VisionLEDMode.kOn);
  }

  private void configureLightingHardware() {
    var hardware = HardwareAdapter.LightingHardware.getInstance();
  }

  /** Updates the hardware to run with output values of {@link SubsystemBase}'s. */
  void writeHardware(Set<SubsystemBase> enabledSubsystems, @ReadOnly RobotState robotState) {
    if (enabledSubsystems.contains(Swerve.getInstance())) updateSwerve();
    Robot.sLoopDebugger.addPoint("updateSwerve");
    if (enabledSubsystems.contains(Intake.getInstance())) updateIntake();
    Robot.sLoopDebugger.addPoint("updateIntake");
    if (enabledSubsystems.contains(Indexer.getInstance())) updateIndexer();
    Robot.sLoopDebugger.addPoint("updateIndex");
    if (enabledSubsystems.contains(Shooter.getInstance())) updateShooter();
    Robot.sLoopDebugger.addPoint("updateShooter");
    if (enabledSubsystems.contains(Climber.getInstance())) updateClimber();
    Robot.sLoopDebugger.addPoint("updateClimb");
    if (enabledSubsystems.contains(Lighting.getInstance())) updateLighting();
    Robot.sLoopDebugger.addPoint("updateLight");
    updateJoysticks();
    Robot.sLoopDebugger.addPoint("updateJoy");
  }

  private void updateSwerve() {
    var hardware = HardwareAdapter.SwerveHardware.getInstance();
    var miscHardware = HardwareAdapter.MiscHardware.getInstance();

    var outputs = Swerve.getInstance().getOutputs();
    if (Swerve.getInstance().getZero()) miscHardware.gyro.zeroYaw();
    if (!outputs.isIdle()) {
      hardware.FL.set(outputs.voltages()[0], outputs.steerAngles()[0]);
      hardware.FR.set(outputs.voltages()[1], outputs.steerAngles()[1]);
      hardware.BL.set(outputs.voltages()[2], outputs.steerAngles()[2]);
      hardware.BR.set(outputs.voltages()[3], outputs.steerAngles()[3]);
    }
  }

  private void updateIndexer() {
    var hardware = HardwareAdapter.IndexerHardware.getInstance();
    var intakeIndexerOutput = Indexer.getInstance().getIntakeIndexerOutput();
    var kickerOutput = Indexer.getInstance().getKickerOutput();
    hardware.kickerMotor.setOutput(kickerOutput, false);
    hardware.intakeIndexerMotor.setOutput(intakeIndexerOutput);
  }

  private void updateIntake() {
    var hardware = HardwareAdapter.IntakeHardware.getInstance();
    var output = Intake.getInstance().getOutputs();
    var intakeArmOutput1 = IntakeArm.getInstance().getOutputs1();
    var intakeArmOutput2 = IntakeArm.getInstance().getOutputs2();

    hardware.motor.setOutput(output, false);
    hardware.armMotorRight.setOutput(intakeArmOutput1, false);
    hardware.armMotorLeft.setOutput(intakeArmOutput2, false);
  }

  private void updateShooter() {
    var hardware = HardwareAdapter.ShooterHardware.getInstance();

    var shooterOutput = Shooter.getInstance().getShooterOutputs();
    var hoodOutput = Shooter.getInstance().getHoodOutputs();

    SmartDashboard.putNumber("rpmW", shooterOutput.getReference() / 2048.0 * 600.0);
    hardware.shooterMasterMotor.setOutput(shooterOutput, false);
    hardware.hoodMotor.setOutput(hoodOutput, false);
  }

  private void updateClimber() {
    var hardware = HardwareAdapter.ClimberHardware.getInstance();
    var outputs = Climber.getInstance().getOutputs();

    if (outputs.leftEncTicks == ClimberConstants.leftDownTicks - 6000) {
      hardware.leftPuller.config_kP(0, ClimberConstants.pullerGains.p + 1.82);
      hardware.leftPuller.config_kF(0, ClimberConstants.pullerGains.f + 1.85);
    }

    hardware.leftPuller.set(ControlMode.MotionMagic, outputs.leftEncTicks);
    hardware.rightPuller.set(ControlMode.MotionMagic, outputs.rightEncTicks);
  }

  public void updateLighting() {
    var hardware = HardwareAdapter.LightingHardware.getInstance();
    hardware.candle.animate(Lighting.getInstance().getOutput());
  }

  private void updateJoysticks() {}
}
