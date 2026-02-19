package frc.robot.subsystems.shooter;

import java.util.Map;
import java.util.TreeMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterWheelsL;
    private final SparkMax shooterWheelsF;
    private final SparkMax shooterPivot;
    private static int shooterWheelsLCanId = 8;
    private static int shooterWheelsFCanId = 9;
   // private static int shooterPivotCanId = -1;
    public static double shooterPowerRT = 0.85;
    public static double shooterPowerB = 0.5;
    public static double shooterPowerA = 0.8;
    public static double shooterPowerX = 0.9;
    public static double shooterPowerY = 0.15;

    public static double IdealShootSpeed = 0; //rotations per minute
    //public boolean GoodSpeed = UpToSpeed();
  
    final XboxController manipulatorXbox = new XboxController(1);

    
    PIDController pid = new PIDController (0,0,0);
    public ShooterSubsystem(){
        shooterWheelsL = new SparkMax(shooterWheelsLCanId,MotorType.kBrushless);
        shooterWheelsF = new SparkMax(shooterWheelsFCanId,MotorType.kBrushless);
        shooterPivot = null;// new SparkMax(shooterPivotCanId,MotorType.kBrushless);
        double ShooterVelocity = shooterWheelsL.getAbsoluteEncoder().getVelocity();
        
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
      }

  public void setPower(double power) {
    shooterWheelsL.set(power);
  }

  public void stop() {  
      shooterWheelsL.set(0);
  }
    
    public void shootRT() { 
      System.out.println("it's uh shooting with RT");
      System.out.println(manipulatorXbox.getRightTriggerAxis() * 100);
      shooterWheelsL.set(manipulatorXbox.getRightTriggerAxis());
      shooterWheelsF.set(manipulatorXbox.getRightTriggerAxis());
    }

    public void shootB() {
      System.out.println("it's uh shooting with B");
      shooterWheelsL.set(shooterPowerB);
      shooterWheelsF.set(shooterPowerB);
    }

    public void shootA() {
      System.out.println("it's uh shooting with A");
      shooterWheelsL.set(shooterPowerA);
      shooterWheelsF.set(shooterPowerA);
    }

    public void shootX() {
      System.out.println("it's uh shooting with X"); ////
        shooterWheelsL.set(shooterPowerX); 
        shooterWheelsF.set(shooterPowerX); 
    }

    public void shootY() {
      System.out.println("it's uh shooting with Y");
      shooterWheelsL.set(shooterPowerY);
      shooterWheelsF.set(shooterPowerY);
    }

    public void stopShooting(){
      shooterWheelsL.set(0);
      shooterWheelsF.set(0);
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