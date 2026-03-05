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


public class CollectorSubsystem extends SubsystemBase {
     private static final int COLLECTOR_WHEELS_L_CAN_ID = 21;
     private static final int COLLECTOR_WHEELS_F_CAN_ID = 22;
     private static final int COLLECTOR_PIVOT_L_CAN_ID = 14;
     private static final int COLLECTOR_PIVOT_F_CAN_ID = 15;  
    private static final double COLLECT_POWER = 0.7;    

    private final SparkMax collectorWheelsL;
    private final SparkMax collectorWheelsF;
    private final SparkMax collectorPivotL;
    private final SparkMax collectorPivotF;
    
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);

    private boolean pivotUp = true;

    public CollectorSubsystem() {
         collectorWheelsL = new SparkMax(COLLECTOR_WHEELS_L_CAN_ID, MotorType.kBrushless);
         collectorWheelsF = new SparkMax(COLLECTOR_WHEELS_F_CAN_ID, MotorType.kBrushless);
        collectorPivotL = new SparkMax(COLLECTOR_PIVOT_L_CAN_ID, MotorType.kBrushed);
        collectorPivotF = new SparkMax(COLLECTOR_PIVOT_F_CAN_ID, MotorType.kBrushed);

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
          .apply(globalConfig);
          //.follow(collectorWheelsL, true);
          
          collectorPivotL.configure(collectorPivotLConfig, ResetMode.kResetSafeParameters,
           PersistMode.kPersistParameters);
           collectorPivotF.configure(collectorPivotFConfig, ResetMode.kResetSafeParameters,
           PersistMode.kPersistParameters);
            collectorWheelsL.configure(collectorWheelsLConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
          collectorWheelsF.configure(collectorWheelsFConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);        
    }

    @Override
    public void periodic() {
        if (pivotUp == true) {
            final boolean topLimitSwitchValue = toplimitSwitch.get();
            //collectorPivotL.set(topLimitSwitchValue ? -0.25 : 0);
        }
        else {
            final boolean bottomLimitSwitchValue = bottomlimitSwitch.get();
          //  collectorPivotL.set(bottomLimitSwitchValue ? 0.25 : 0);
        }
    }

     public void startCollecting() {
         collectorWheelsL.set(COLLECT_POWER);
     }

     public void stopCollecting() {
         collectorWheelsL.set(0);
     }

     public void startReleasing() {
         collectorWheelsL.set(-COLLECT_POWER);
     }

    // has to pivot more than 90 degrees
    public void pivotCollectorDown() {
        pivotUp = false;
    }

    public void pivotCollectorUp() {
        pivotUp = true;
    }
}