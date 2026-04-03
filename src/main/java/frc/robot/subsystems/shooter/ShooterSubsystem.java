package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  // Shooter flywheel constants
  private static final double DESIRED_SHOOTING_VELOCITY = -56.0; // The desired rotations-per-second to run the shooter when shooting
  private static final double DESIRED_PASSING_VELOCITY = -45.0; // The desired rotations-per-second to run the shooter when passing
  private static final double DESIRED_REVERSE_VELOCITY = 60.0; // The desired rotations-per-second to run the shooter when in reverse

  private static final double MINIMUM_SHOOTING_VELOCITY = -53.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode
  private static final double MINIMUM_PASSING_VELOCITY = -28.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode

  // Shooter flywheel feedforward/feedback constants
  private static final double FLYWHEEL_P = 20.0f; // TODO
  private static final double FLYWHEEL_I = 0.0f; // TODO: Will probably stay at 0
  private static final double FLYWHEEL_D = 0.0f; // TODO: Might get changes to prevent overshooting, tune after V

  private static final double FLYWHEEL_S = 0.0; // TODO: This is for overcoming static friction, may not be needed tbh
  private static final double FLYWHEEL_V = 1.5; // TODO: From quick research makes me believe this is the feedforward value we care about

  // Shooter pivot constants
  public static final double PIVOT_INCREMENT_MANUAL = 0.4;

  private static final double PIVOT_DOWN = 0.0;
  private static final double PIVOT_UP = -0.11; 
  private static final double PIVOT_PASS = -1.257;

  // Can IDs
  private static final int FLYWHEELS_L_CAN_ID = 0;
  private static final int FLYWHEELS_F_CAN_ID = 1;
  private static final int PIVOT_CAN_ID = 4;

  // External controls
  private DoubleSupplier manualPivotInput = null;

  // Motor
  private final TalonFX flywheelLeader = new TalonFX(FLYWHEELS_L_CAN_ID);
  private final TalonFX shooterFollower = new TalonFX(FLYWHEELS_F_CAN_ID);
  private final Follower flywheelFollower = new Follower(FLYWHEELS_L_CAN_ID, MotorAlignmentValue.Opposed);
  private final SparkMax pivot = new SparkMax(PIVOT_CAN_ID, MotorType.kBrushed);

  // Flywheel variables
  private final NeutralOut brakeControl = new NeutralOut();
  private final VelocityVoltage flywheelVelocityVoltage = new VelocityVoltage(DESIRED_SHOOTING_VELOCITY).withSlot(0);
  private final StatusSignal<AngularVelocity> flywheelVelocitySignal = flywheelLeader.getVelocity();
  
  private double currentMinimumFlywheelVelocity = 0.0f; // The current minimum velocity required for the flywheel to be considerd "up-to-speed"

  // POINT OF VARIABLES THAT HAVENT BEEN SORTED

  // TODO: Potentially remove this and just use setpoints directly
  //        . Could also put the setpoints with these and still drive them?
  public enum PivotState {
    ThreeMeters, TwoMeters, manual
  }
  public PivotState desiredPivotState = PivotState.TwoMeters;

  @Override
  public void periodic() {
    // Refresh the velocity signal so the current flywheel velocity can be measured. THIS ALWAYS NEEDS TO BE RUN! (it caches the value)
    flywheelVelocitySignal.refresh();
   // System.out.println("Flywheel speed: " + flywheelVelocitySignal.getValueAsDouble());

    // Manually control the hood pivot (if desired) by incrmenetally changing the setpoint
    if (desiredPivotState != PivotState.manual) {
      return;
    }

    final SparkClosedLoopController closedLoopController = pivot.getClosedLoopController();
    final double currentSetpoint = closedLoopController.getSetpoint();
    final double movementInput = manualPivotInput.getAsDouble();
    final double newSetpoint = currentSetpoint + (movementInput / 20);

    closedLoopController.setSetpoint(newSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public ShooterSubsystem() {
    SparkMaxConfig pivotConfig = (SparkMaxConfig) new SparkMaxConfig()
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);

      AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig()
        .countsPerRevolution(280);

      ClosedLoopConfig pivotClosedLoopConfig = new ClosedLoopConfig()
        .pid(0.85, 0.0, 0.0, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .positionWrappingEnabled(false)
        .outputRange(-1, 1);

    pivotConfig.apply(encoderConfig);
    pivotConfig.apply(pivotClosedLoopConfig);

    pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // TODO: Properly configured the feedback and feedforward controller of the leader TalonFX
    shooterFollower.setControl(flywheelFollower);
    TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(new Slot0Configs()
          .withKP(FLYWHEEL_P)
          .withKI(FLYWHEEL_I)
          .withKD(FLYWHEEL_D)
          .withKS(FLYWHEEL_S) // NOTE: Both of there were not originally present, these are the feedforward terms
          .withKV(FLYWHEEL_V) // NOTE: Both of there were not originally present, these are the feedforward terms
        );

    flywheelLeader.getConfigurator().apply(configuration);
    shooterFollower.getConfigurator().apply(configuration);

    // Initially set the pivot position to be the bottom positon
    pivot.getClosedLoopController().setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setShooterWheelsShoot() {
    flywheelLeader.setControl(flywheelVelocityVoltage.withVelocity(DESIRED_SHOOTING_VELOCITY));
    currentMinimumFlywheelVelocity = MINIMUM_SHOOTING_VELOCITY;
  }

  public void setShooterWheelsPass() {
    flywheelLeader.setControl(flywheelVelocityVoltage.withVelocity(DESIRED_PASSING_VELOCITY));
    currentMinimumFlywheelVelocity = MINIMUM_PASSING_VELOCITY;
  }

  public void setShooterWheelsReverse() {
    flywheelLeader.setControl(flywheelVelocityVoltage.withVelocity(DESIRED_REVERSE_VELOCITY));
    currentMinimumFlywheelVelocity = 0.0f;
  }

  public void stopShooterWheels() {
    flywheelLeader.setControl(brakeControl);
    // TODO: Find a better way to do this, maybe a stack?
    currentMinimumFlywheelVelocity = MINIMUM_SHOOTING_VELOCITY;
  }

  // TODO: Potentially put this as an enum and use just one pivot method (for named pivot spots)?
  public void setPivotToPass() {
    pivotToPosition(PIVOT_PASS);
  }

  // TODO: Potentially put this as an enum and use just one pivot method (for named pivot spots)?
  public void setPivotToShoot() {
    pivotToPosition(PIVOT_DOWN);
  }

  public void setPivotManually() {
    desiredPivotState = PivotState.manual;
    updateShooterPivot();
  }
  
  public void pivotUpManually() {
    pivot.set(-PIVOT_INCREMENT_MANUAL);
  }

  public void pivotDownManually() {
    pivot.set(PIVOT_INCREMENT_MANUAL);
  }

  public void stopPivotingManually() {
    pivot.set(0.0f);
  }

  // NOTE: This assumes all angles are negative (hence the <=), this might need to change in the future
  public boolean isFlywheelUpToSpeed() {
    //System.out.println("Minimum Velocity: " + currentMinimumFlywheelVelocity);
    return flywheelVelocitySignal.getValueAsDouble() <= currentMinimumFlywheelVelocity;
  }

  public void printPivotAngle() {
   // System.out.println("Current Pivot Angle: " + pivot.getAlternateEncoder().getPosition());
  }

  // TODO: Consider making this private?
  public void pivotToPosition(final double position){
    pivot.getClosedLoopController().setSetpoint(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // TODO: Potentially remove this when pivot state gets removed
  private void updateShooterPivot() {
    final double PIVOT_POSITON = (desiredPivotState == PivotState.TwoMeters) ? PIVOT_DOWN : PIVOT_UP;
    pivot.getClosedLoopController().setSetpoint(PIVOT_POSITON, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  // Methods below this point have not been organized

  // TODO: Review whether this should stay
  //        . If it does stay, dont modify the constant (split into separate variable)
  // public void changeSpeedUp() {
  //   // Limits speed so it doesn't go below -1.0
  //   if (SHOOTERSPEED > -1.0) {
  //     SHOOTERSPEED -= 0.05;
  //   }
  // }

  // TODO: Review whether this should stay
  //        . If it does stay, dont modify the constant (split into separate variable)
  // public void changeSpeedDown() {
  //   // Limits speed so it doesn't go above 0
  //   SHOOTERSPEED += 0.05;
  // }

  private double calculateHighArcAngle(final double distance, final double velocity, final double targetHeight) {
    final double g = 9.80665;

    final double v2 = Math.pow(velocity, 2);
    final double v4 = Math.pow(velocity, 4);
    final double x2 = Math.pow(distance, 2);

    // Formula: v^4 - g(g*x^2 + 2*y*v^2)
    final double rootContent = v4 - g * (g * x2 + 2 * targetHeight * v2);

    if (rootContent < 0) {
      return Double.NaN;
    }

    final double tanThetaHigh = (v2 + Math.sqrt(rootContent)) / (g * distance);

    return Math.toDegrees(Math.atan(tanThetaHigh));
  }
}