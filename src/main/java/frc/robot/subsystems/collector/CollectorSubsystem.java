package frc.robot.subsystems.collector;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DebugHelpers;

public class CollectorSubsystem extends SubsystemBase {
    private final SparkMax collectorWheelsL;
    private final SparkMax collectorWheelsF;
    private final SparkMax collectorPivotL;
    private final SparkMax collectorPivotF;
    private static int collectorWheelsLCanId = 8;
    private static int collectorWheelsFCanId = 9;
    private static int collectorPivotLCanId = 14;
    private static int collectorPivotFCanId = 15;  
    private static double collectPower = 0.7;
    private static double uncollectPower = 0.7;
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private boolean pivotUp = true;



    PIDController pid = new PIDController(0, 0, 0);

    public CollectorSubsystem() {
        collectorWheelsL = new SparkMax(collectorWheelsLCanId, MotorType.kBrushless);
        collectorWheelsF = new SparkMax(collectorWheelsFCanId, MotorType.kBrushless);
        collectorPivotL = new SparkMax(collectorPivotLCanId, MotorType.kBrushed);
        collectorPivotF = new SparkMax(collectorPivotFCanId, MotorType.kBrushed);

        
          SparkMaxConfig globalConfig = new SparkMaxConfig();
          SparkMaxConfig collectorPivotLConfig = new SparkMaxConfig();
          SparkMaxConfig collectorPivotFConfig = new SparkMaxConfig();
          SparkMaxConfig collectorWheelsLConfig = new SparkMaxConfig();
          SparkMaxConfig collectorWheelsFConfig = new SparkMaxConfig();
          
          globalConfig
          .smartCurrentLimit(50)
          .idleMode(IdleMode.kBrake);
          
          collectorPivotLConfig
          .apply(globalConfig)
          .inverted(true);    
               collectorPivotFConfig
          .apply(globalConfig)
          .follow(collectorPivotL, true);
          
        collectorWheelsLConfig
          .apply(globalConfig)
          .inverted(false);
          
          collectorWheelsFConfig
          .apply(globalConfig)
          .follow(collectorWheelsL, true);
          

          collectorPivotL.configure(collectorPivotLConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
          collectorPivotF.configure(collectorPivotFConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
            collectorWheelsL.configure(collectorWheelsLConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
          collectorWheelsF.configure(collectorWheelsFConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
         
    }

    public void collect() {
        collectorWheelsL.set(collectPower);
    }

    public void stopCollect() {
        collectorWheelsL.set(0);
    }

    public void uncollect() {
        collectorWheelsL.set(-uncollectPower);
    }

    // has to pivot more than 90 degrees
    public void collectorPivot() {
        /*DebugHelpers.printMagneticLimitSwitchValue("Bottom limit switch value: ", bottomlimitSwitch);
        DebugHelpers.printMagneticLimitSwitchValue("Top limit switch value: ", toplimitSwitch);

        boolean bottomLimitSwitchValue =  bottomlimitSwitch.get();
        if (bottomLimitSwitchValue == true) {
            collectorPivotL.set(0);
        } else {
            collectorPivotL.set(0.1);
        }*/

        pivotUp = false;
    }

    @Override
    public void periodic() {
        if (pivotUp == true) {
//            DebugHelpers.printMagneticLimitSwitchValue("Limit Switch Val top: ", toplimitSwitch);
            boolean topLimitSwitchValue = toplimitSwitch.get();
            collectorPivotL.set(topLimitSwitchValue ? -0.25 : 0);
            //collectorPivotF.set(topLimitSwitchValue ? -0.25 : 0);

        }
        else {
//            DebugHelpers.printMagneticLimitSwitchValue("Limit Switch Val bottom: ", bottomlimitSwitch);
            boolean bottomLimitSwitchValue = bottomlimitSwitch.get();
            collectorPivotL.set(bottomLimitSwitchValue ? 0.25 : 0);
            //collectorPivotF.set(bottomLimitSwitchValue ? 0.25 : 0);
        }
    }

    public void stopCollectorPivot() {
        /*boolean topLimitSwitchValue =  toplimitSwitch.get();
        if (topLimitSwitchValue == true) {
            collectorPivotL.set(0);
        } else {
            collectorPivotL.set(-0.1);
        }*/

        pivotUp = true;
    }
}