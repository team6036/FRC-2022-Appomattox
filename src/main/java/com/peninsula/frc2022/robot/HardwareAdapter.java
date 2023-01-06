package com.peninsula.frc2022.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.peninsula.frc2022.config.PortConstants;
import com.peninsula.frc2022.config.SwerveConstants;
import com.peninsula.frc2022.util.config.FalconFactory;
import com.peninsula.frc2022.util.config.TalonFXFactory;
import com.peninsula.frc2022.util.control.Falcon;
import com.peninsula.frc2022.util.control.Spark;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.*;
import org.photonvision.PhotonCamera;

/**
 * Represents all hardware components of the robot. Singleton class. Should only be used in robot
 * package. Subdivides hardware into subsystems.
 */
public class HardwareAdapter {

  /** 1 NavX */
  public static class MiscHardware {
    private static MiscHardware sInstance;
    public final AHRS gyro;

    private MiscHardware() {
      gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    }

    static MiscHardware getInstance() {
      if (sInstance == null) sInstance = new MiscHardware();
      return sInstance;
    }
  }

  /** 4 Falcon 500s (controlled by Talon FX) */
  public static class SwerveHardware {

    private static SwerveHardware sInstance;

    // CW Starting from top left module
    // <Drive Motor, Turn Motor, Encoder>
    public final SwerveModule FL, FR, BL, BR;
    public final SwerveModule[] modules;

    private SwerveHardware() {

      FL =
          Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
              PortConstants.driveFL.kD,
              PortConstants.driveFL.kT,
              PortConstants.driveFL.kE,
              -SwerveConstants.FLo);
      FR =
          Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
              PortConstants.driveFR.kD,
              PortConstants.driveFR.kT,
              PortConstants.driveFR.kE,
              -SwerveConstants.FRo);
      BL =
          Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
              PortConstants.driveBL.kD,
              PortConstants.driveBL.kT,
              PortConstants.driveBL.kE,
              -SwerveConstants.BLo);
      BR =
          Mk3SwerveModuleHelper.createFalcon500(
              Mk3SwerveModuleHelper.GearRatio.FAST_WITH_TREAD,
              PortConstants.driveBR.kD,
              PortConstants.driveBR.kT,
              PortConstants.driveBR.kE,
              -SwerveConstants.BRo);
      modules = new SwerveModule[] {FL, FR, BL, BR};
    }

    static SwerveHardware getInstance() {
      if (sInstance == null) sInstance = new SwerveHardware();
      return sInstance;
    }
  }

  /** 1 CANdle */
  static class LightingHardware {

    private static LightingHardware sInstance;

    final CANdle candle;

    private LightingHardware() {
      candle = new CANdle(PortConstants.kLighting);
    }

    static LightingHardware getInstance() {
      if (sInstance == null) sInstance = new LightingHardware();
      return sInstance;
    }
  }

  /** 3 Falcons for arm + rotation */
  static class IntakeHardware {

    private static IntakeHardware sInstance;

    final Falcon motor;
    final Falcon armMotorLeft;
    final Falcon armMotorRight;

    private IntakeHardware() {

      motor = FalconFactory.createDefaultTalon(PortConstants.kIntakeId, "intake");
      armMotorLeft =
          FalconFactory.createDefaultTalon(PortConstants.kIntakeArmMotorLeftID, "armLeft");
      armMotorRight =
          FalconFactory.createDefaultTalon(PortConstants.kIntakeArmMotorRightID, "armRight");
    }

    static IntakeHardware getInstance() {
      if (sInstance == null) sInstance = new IntakeHardware();
      return sInstance;
    }
  }

  /** 1 NEO for intake indexer motor 1 Falcon for kicker wheel motor */
  static class IndexerHardware {

    private static IndexerHardware sInstance;

    final Spark intakeIndexerMotor;
    final Falcon kickerMotor;
    final DigitalInput intakeIndexerSensor;
    final DigitalInput stopperSensor;
    final DigitalInput firstPosSensor;

    private IndexerHardware() {
      intakeIndexerMotor = new Spark(PortConstants.kIntakeIndexerId, "intakeIndexer");

      intakeIndexerSensor = new DigitalInput(PortConstants.kIntakeIndexerSensorId);
      stopperSensor = new DigitalInput(PortConstants.kStopperSensorId);
      firstPosSensor = new DigitalInput(PortConstants.kFirstPosSensorId);

      kickerMotor = FalconFactory.createDefaultTalon(PortConstants.kKickerId, "kicker");
    }

    static IndexerHardware getInstance() {
      if (sInstance == null) sInstance = new IndexerHardware();
      return sInstance;
    }
  }

  /** 3 Falcons for shooter main + hood */
  static class ShooterHardware {

    private static ShooterHardware sInstance;
    Falcon shooterMasterMotor;
    Falcon shooterSlaveMotor;
    Falcon hoodMotor;

    private ShooterHardware() {
      shooterMasterMotor =
          FalconFactory.createDefaultTalon(PortConstants.kShooterMaster, "shooterMaster");
      shooterSlaveMotor =
          FalconFactory.createDefaultTalon(PortConstants.kShooterSlave, "shooterSlave");
      hoodMotor = FalconFactory.createDefaultTalon(PortConstants.kShooterHood, "hoodSlave");
    }

    static ShooterHardware getInstance() {
      if (sInstance == null) sInstance = new ShooterHardware();
      return sInstance;
    }
  }

  /** 2 Falcons (one for each arm) */
  static class ClimberHardware {

    private static ClimberHardware sInstance;
    final TalonFX rightPuller, leftPuller;

    private ClimberHardware() {
      rightPuller =
          TalonFXFactory.createDefaultTalon(PortConstants.kClimberRightPuller, "rightPuller");
      leftPuller =
          TalonFXFactory.createDefaultTalon(PortConstants.kClimberLeftPuller, "leftPuller");
    }

    static ClimberHardware getInstance() {
      if (sInstance == null) sInstance = new ClimberHardware();
      return sInstance;
    }
  }

  /** 1 Photon Camera */
  static class VisionHardware {

    private static VisionHardware sInstance;

    PhotonCamera camera;

    private VisionHardware() {
      camera = new PhotonCamera("gloworm");
    }

    static VisionHardware getInstance() {
      if (sInstance == null) sInstance = new VisionHardware();
      return sInstance;
    }
  }

  /** 2 Xbox controllers */
  static class JoystickHardware {

    private static JoystickHardware sInstance;

    final XboxController driverXboxController;
    final XboxController operatorXboxController;

    private JoystickHardware() {
      driverXboxController = new XboxController(PortConstants.kDriverId);
      operatorXboxController = new XboxController(PortConstants.kOperatorId);
    }

    static JoystickHardware getInstance() {
      if (sInstance == null) sInstance = new JoystickHardware();
      return sInstance;
    }
  }

  private HardwareAdapter() {}
}
