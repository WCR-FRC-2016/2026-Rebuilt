package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterWheelsM;
    private final SparkMax shooterWheelsS;
    private final SparkMax shooterPivot;
    private static int shooterWheelsMCanId = -1;
    private static int shooterWheelsSCanId = -1;
    private static int shooterPivotCanId = -1;
    public static double shooterPower = -1;
    public static double IdealShootSpeed = 0; //rotations per minute
    public boolean GoodSpeed = UpToSpeed();
  
    
    PIDController pid = new PIDController (0,0,0);
    public ShooterSubsystem(){
        shooterWheelsM = new SparkMax(shooterWheelsMCanId,MotorType.kBrushed);
        shooterWheelsS = new SparkMax(shooterWheelsSCanId,MotorType.kBrushed);
        shooterPivot = new SparkMax(shooterPivotCanId,MotorType.kBrushless);
        double ShooterVelocity = shooterWheelsM.getAbsoluteEncoder().getVelocity();
        
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig shooterWheelsMConfig = new SparkMaxConfig();
        SparkMaxConfig shooterWheelsSConfig = new SparkMaxConfig();

        globalConfig
          .smartCurrentLimit(50)
          .idleMode(IdleMode.kBrake);
          
          shooterWheelsMConfig
          .apply(globalConfig)
          .inverted(true);
          
          shooterWheelsSConfig
          .apply(globalConfig)
          .follow(shooterWheelsS);
          
          shooterWheelsM.configure(shooterWheelsMConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
          shooterWheelsS.configure(shooterWheelsSConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters); 

      }
    
    public void shoot(){
      
      shooterWheelsM.set(shooterPower);
    }
    public double ShooterSpeed(){
     return shooterWheelsM.getAbsoluteEncoder().getVelocity(); //Rotations per minute
    }
    
    public boolean UpToSpeed(){
      if (ShooterSpeed()- IdealShootSpeed == 0) {
        return true;
      }
      else {return false;
    }}
    public void shooterAnglePivot(){
      
    }
    }

