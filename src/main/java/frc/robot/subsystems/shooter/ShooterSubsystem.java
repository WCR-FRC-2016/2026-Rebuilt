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
  private static final double DESIRED_REVERSE_VELOCITY = 60.0; // The desired rotations-per-second to run the shooter when in reverse
  private static final double DESIRED_PASSING_VELOCITY = -45.0; // The desired rotations-per-second to run the shooter when passing

  private static final double DESIRED_SHOOTING_VELOCITY = -46.5; // The desired rotations-per-second to run the shooter when shooting
  private static final double DESIRED_SHOOT_CLOSE_VELOCITY = -40.0; // The desired rotations-per-second to run the shooter when close 
  private static final double DESIRED_SHOOT_CORNER_VELOCITY = -63.0; // -60.0 // The desired rotations-per-second to run the shooter when in corner 
  private static final double DESIRED_SHOOT_CLIMBER_VELOCITY = -51.0; // The desired rotations-per-second to run the shooter when at climb tower x:11.65, y: 6.25
  private static final double DESIRED_SHOOT_TRENCH_VELOCITY = -56.0; // The desired rotations-per-second to run the shooter when at climb tower x:11.65, y: 6.25


  private static final double MINIMUM_PASSING_VELOCITY = -28.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode

  private static final double MINIMUM_SHOOTING_VELOCITY = -53.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode
  private static final double MINIMUM_SHOOT_CLOSE_VELOCITY = -35.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode
  private static final double MINIMUM_SHOOT_CORNER_VELOCITY = -60.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode
  private static final double MINIMUM_SHOOT_CLIMBER_VELOCITY = -45.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode
  private static final double MINIMUM_SHOOT_TRENCH_VELOCITY = -45.0; // The minimum velocity before the shooter is considerd "up-to-speed" for this mode


  private static final double FEED_FLYWHEELS_SPEED = -0.7; // the motor power of the things that feed the balls from the agitator to the actual shooting wheels
  private static final double FEED_FLYWHEELS_REVERSE = 0.7; // the motor power of the things that feed the balls from the agitator to the actual shooting wheels

  // Shooter flywheel feedforward/feedback constants
  private static final double FLYWHEEL_P = 0.1f; // TODO
  private static final double FLYWHEEL_I = 0.0f; // TODO: Will probably stay at 0
  private static final double FLYWHEEL_D = 0.0f; // TODO: Might get changes to prevent overshooting, tune after V

  private static final double FLYWHEEL_S = 0.0; // TODO: This is for overcoming static friction, may not be needed tbh
  private static final double FLYWHEEL_V = 0.12; // TODO: From quick research makes me believe this is the feedforward value we care about

  // Shooter pivot constants
  public static final double PIVOT_INCREMENT_MANUAL = 0.4;

  private static final double PIVOT_DOWN = 0.0;
  private static final double PIVOT_UP = -0.11; 
  private static final double PIVOT_PASS = -1.257;

  // Can IDs
  private static final int FLYWHEELS_L_CAN_ID = 0;
  private static final int FLYWHEELS_F_CAN_ID = 1;
  private static final int FEED_FLYWHEELS_CAN_ID = 4;


  // Motor
  private final TalonFX flywheelLeader = new TalonFX(FLYWHEELS_L_CAN_ID);
  private final TalonFX shooterFollower = new TalonFX(FLYWHEELS_F_CAN_ID);
  private final Follower flywheelFollower = new Follower(FLYWHEELS_L_CAN_ID, MotorAlignmentValue.Opposed);

  private final SparkMax feedFlyWheels = new SparkMax(FEED_FLYWHEELS_CAN_ID, MotorType.kBrushless);
  

  // Flywheel variables
  private final NeutralOut brakeControl = new NeutralOut();
  private final VelocityVoltage flywheelVelocityVoltage = new VelocityVoltage(DESIRED_SHOOTING_VELOCITY).withSlot(0);
  private final StatusSignal<AngularVelocity> flywheelVelocitySignal = flywheelLeader.getVelocity();
  
  private double currentMinimumFlywheelVelocity = 0.0f; // The current minimum velocity required for the flywheel to be considerd "up-to-speed"
  private double desiredFlyWheelVelocity = 0;

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
    //System.out.println("Flywheel speed: " + flywheelVelocitySignal.getValueAsDouble());
  }

  public ShooterSubsystem() {
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
  }

  public void setShooterWheelsShoot() {
   setDesiredFlyWheelVelocity(DESIRED_SHOOTING_VELOCITY);
    currentMinimumFlywheelVelocity = MINIMUM_SHOOTING_VELOCITY;
  }

  public void setShooterWheelsPass() {
   setDesiredFlyWheelVelocity(DESIRED_PASSING_VELOCITY);
    currentMinimumFlywheelVelocity = MINIMUM_PASSING_VELOCITY;
  }

  public void setShooterWheelsReverse() {
   setDesiredFlyWheelVelocity(DESIRED_REVERSE_VELOCITY);
    currentMinimumFlywheelVelocity = 0.0f;
  }
  public void setShooterWheelsShootClose(){
    setDesiredFlyWheelVelocity(DESIRED_SHOOT_CLOSE_VELOCITY);
    currentMinimumFlywheelVelocity = MINIMUM_SHOOT_CLOSE_VELOCITY;
  }
    public void setShooterWheelsShootTrench(){
    setDesiredFlyWheelVelocity(DESIRED_SHOOT_TRENCH_VELOCITY);
    currentMinimumFlywheelVelocity = MINIMUM_SHOOT_TRENCH_VELOCITY;
  }
   public void setShooterWheelsShootCorner(){
    setDesiredFlyWheelVelocity(DESIRED_SHOOT_CORNER_VELOCITY);
    currentMinimumFlywheelVelocity = MINIMUM_SHOOT_CORNER_VELOCITY;
  }
   public void setShooterWheelsShootClimber(){
    setDesiredFlyWheelVelocity(DESIRED_SHOOT_CLIMBER_VELOCITY);
    currentMinimumFlywheelVelocity = MINIMUM_SHOOT_CLIMBER_VELOCITY;
  }
  public void setDesiredFlyWheelVelocity(final double newDesiredFlyWheelVelocity) {
    desiredFlyWheelVelocity = newDesiredFlyWheelVelocity;
    flywheelLeader.setControl(flywheelVelocityVoltage.withVelocity(newDesiredFlyWheelVelocity));
  }

  public void stopShooterWheels() {
    flywheelLeader.setControl(brakeControl);
    // TODO: Find a better way to do this, maybe a stack?
    currentMinimumFlywheelVelocity = MINIMUM_SHOOTING_VELOCITY;
  }
  // NOTE: This assumes all angles are negative (hence the <=), this might need to change in the future
  public boolean isFlywheelUpToSpeed() {
    //System.out.println("Minimum Velocity: " + currentMinimumFlywheelVelocity);
    return flywheelVelocitySignal.getValueAsDouble() <= currentMinimumFlywheelVelocity;
  }

  public void feedFlyWheels(){
    feedFlyWheels.set(FEED_FLYWHEELS_SPEED);
  }
public void reverseFeedFlyWheels(){
    feedFlyWheels.set(FEED_FLYWHEELS_REVERSE);
  }

 public void feedFlyWheelsStop(){
    feedFlyWheels.set(0);
  }

}