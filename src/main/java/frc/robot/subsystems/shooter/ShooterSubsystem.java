package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Amps;


public class ShooterSubsystem extends SubsystemBase {
  private static int shooterWheelsLCanId = 0;
  private static int shooterWheelsFCanId = 1;
  private static int pivotWheelCanId = 4;

  public static double IdealShootSpeed = 0; // rotations per minute
  // public boolean GoodSpeed = UpToSpeed();
  public double SHOOTERSPEED = -0.7;// currently runs at 6.2 perfect for 2.5 distance
  public final double PIVOTSPEED = 0.4;

  int wantedVelocity = 60;

  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(wantedVelocity).withSlot(0);
  private final NeutralOut m_brake = new NeutralOut();


  private final TalonFX shooterLeader = new TalonFX(shooterWheelsLCanId);
  private final Follower follower = new Follower(shooterWheelsLCanId, MotorAlignmentValue.Opposed);
  private final TalonFX shooterFollower = new TalonFX(shooterWheelsFCanId);
  private final SparkMax pivotWheel = new SparkMax(pivotWheelCanId, MotorType.kBrushed);

  private final StatusSignal<AngularVelocity> velocitySignal = shooterLeader.getVelocity();

  @Override
  public void periodic() {
      System.out.println(velocitySignal.getValueAsDouble()); 
      velocitySignal.refresh(); 
  }
  
  /*
   * private DoubleSupplier getRampSpeed = null;
   * private double currentTriggerValue = 0;
   * private PIDController pid = new PIDController(0.1, 0, 0.01);
   * private double rampedSetpoint = 0;
   * private double motorOutput = 0;
   * private int currentMotorVelocity = 0;
   */
  public ShooterSubsystem() {

    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    pivotConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);

    pivotWheel.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterFollower.setControl(follower);
    TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    shooterLeader.getConfigurator().apply(configuration);
    shooterFollower.getConfigurator().apply(configuration);
  }

  public void ShooterWheelsRun() {
    shooterLeader.set(SHOOTERSPEED);

  }

  public void ShooterWheelsRunAuto(double speed) {
    shooterLeader.set(speed);
  }

  public void ShooterWheelsStop() {
    shooterLeader.setControl(m_brake);
  }

  public void pivotUp() {
    pivotWheel.set(-PIVOTSPEED);
  }

  public void pivotDown() {
    pivotWheel.set(PIVOTSPEED);
  }

  public void stopPivotizing() {
    pivotWheel.set(0);
  }

 public void changeSpeedUp() {
    // Limits speed so it doesn't go below -1.0
    if (SHOOTERSPEED > -1.0) { 
        SHOOTERSPEED -= 0.05;
    }
}

public void changeSpeedDown() {
    // Limits speed so it doesn't go above 0
      SHOOTERSPEED += 0.05;
    }


  public double calculateHighArcAngle(double distance, double velocity, double targetHeight) {
    double g = 9.80665;

    double v2 = Math.pow(velocity, 2);
    double v4 = Math.pow(velocity, 4);
    double x2 = Math.pow(distance, 2);

    // Formula: v^4 - g(g*x^2 + 2*y*v^2)
    double rootContent = v4 - g * (g * x2 + 2 * targetHeight * v2);

    if (rootContent < 0) {
      return Double.NaN; 
    }

    double tanThetaHigh = (v2 + Math.sqrt(rootContent)) / (g * distance);

    return Math.toDegrees(Math.atan(tanThetaHigh));
  }
}