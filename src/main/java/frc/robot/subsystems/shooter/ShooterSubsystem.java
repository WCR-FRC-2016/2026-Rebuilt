package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Amps;


public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterWheelsL;
    private final SparkMax shooterWheelsF;
    private final SparkMax pivotWheel;
    private static int shooterWheelsLCanId = 8;
    private static int shooterWheelsFCanId = 9;
    private static int pivotWheelCanId = 7;


    public static double IdealShootSpeed = 0; //rotations per minute
    //public boolean GoodSpeed = UpToSpeed();

    final XboxController manipulatorXbox = new XboxController(1);
    double currentTriggerValue = 0;
    PIDController pid = new PIDController (0.1,0,0.01);
    private final SlewRateLimiter limiter = new SlewRateLimiter(1.0);
    private double currentTriggerTarget = 0;
    double rampedSetpoint = 0;
    double motorOutput = 0;
    int currentMotorVelocity = 0;
    //shooterWheelVelocity = shooterWheelsL.getVelocity();
    private final TalonFX shooterLeader = new TalonFX(shooterWheelsLCanId);
    private final Follower follower = new Follower(21, MotorAlignmentValue.Opposed);
    private final TalonFX shooterFollower = new TalonFX(shooterWheelsFCanId);
    private final StatusSignal<AngularVelocity> velocitySignal = shooterLeader.getVelocity();


    

    public void periodic() {
        currentTriggerTarget = manipulatorXbox.getRightTriggerAxis() * 100;
        rampedSetpoint = limiter.calculate(currentTriggerTarget);
        motorOutput = pid.calculate(currentTriggerValue, rampedSetpoint);
        shooterWheelsL.set(motorOutput);
        velocitySignal.refresh();
      }



    public ShooterSubsystem(){
        shooterWheelsL = new SparkMax(shooterWheelsLCanId,MotorType.kBrushless);
        shooterWheelsF = new SparkMax(shooterWheelsFCanId,MotorType.kBrushless);
        pivotWheel = new SparkMax(pivotWheelCanId,MotorType.kBrushed);
        
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig shooterWheelsLConfig = new SparkMaxConfig();
        SparkMaxConfig shooterWheelsFConfig = new SparkMaxConfig();

        globalConfig
          .smartCurrentLimit(50)
          .idleMode(IdleMode.kBrake);
          
        shooterWheelsLConfig
          .apply(globalConfig)
          .inverted(true);
          
        shooterWheelsFConfig
          .apply(globalConfig)
          .follow(shooterWheelsL, true);

        shooterWheelsL.configure(shooterWheelsLConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
        shooterWheelsF.configure(shooterWheelsFConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters); 

        shooterFollower.setControl(follower);
        TalonFXConfiguration configuration = new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(40)).withStatorCurrentLimitEnable(true)).withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        shooterLeader.getConfigurator().apply(configuration);
        shooterFollower.getConfigurator().apply(configuration);
    }
      
    public double ShooterWheelsRun(double speed) {
        shooterLeader.set(speed);
        double x = shooterLeader.getVelocity().getValueAsDouble();
        return x;
    }

    public Command ShooterWheelsStop() {
        shooterLeader.set(0);
                return null;
    }

    public void pivotUp(double speed) {
      pivotWheel.set(speed);
    }

    public void pivotDown(double speed) {
      pivotWheel.set(speed);
    }

   
    public void stopPivotizing(){
      pivotWheel.set(0);
   }

    public double calculateHighArcAngle(double distance, double velocity, double targetHeight) {
      double g = 9.80665;
    
    // Calculate the core components of the trajectory formula
      double v2 = Math.pow(velocity, 2); //2 meaning squared
      double v4 = Math.pow(velocity, 4);
      double x2 = Math.pow(distance, 2);
    
    // The term inside the square root determines if the target is reachable
    // Formula: v^4 - g(g*x^2 + 2*y*v^2)
    double rootContent = v4 - g * (g * x2 + 2 * targetHeight * v2);
    
    if (rootContent < 0) {
        return Double.NaN; // Target is out of range for this velocity
    }
    
    // Solving for tan(theta) using the derived projectile formula:
    // tan(theta) = (v^2 ± sqrt(v^4 - g(gx^2 + 2yv^2))) / (gx)
    
    // Use the plus (+) for the high arc (the "lob" shot)
    double tanThetaHigh = (v2 + Math.sqrt(rootContent)) / (g * distance);
    
    return Math.toDegrees(Math.atan(tanThetaHigh));
  }
}