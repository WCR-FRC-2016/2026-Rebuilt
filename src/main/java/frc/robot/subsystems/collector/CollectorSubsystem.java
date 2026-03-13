package frc.robot.subsystems.collector;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CollectorSubsystem extends SubsystemBase {
    final CommandXboxController manipulatorCommandXbox = new CommandXboxController(1);
    final XboxController manipulatorXbox = new XboxController(1);

    private static final int COLLECTOR_WHEELS_L_CAN_ID = 9;
    private static final int COLLECTOR_WHEELS_F_CAN_ID = 8;
    private static final int COLLECTOR_PIVOT_L_CAN_ID = 14;
    private static final int COLLECTOR_PIVOT_F_CAN_ID = 15;
    private static final double COLLECT_POWER = 0.6;

    private final SparkMax collectorWheelsL;
    private final SparkMax collectorWheelsF;
   private final SparkMax collectorPivotL;
    private final SparkMax collectorPivotF;

    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);

    private boolean pivotUpManual = false;

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

    @Override
    public void periodic() {
    }

    public void Collect() {
        pivotCollectorDown();
        collectorWheelsL.set(COLLECT_POWER);
    }

    public void stopCollect() {
        collectorWheelsL.set(0);
    }

    public void Release() {
        collectorWheelsL.set(-COLLECT_POWER);
    }

    // has to pivot more than 90 degrees
    public void pivotCollectorDown() {
        collectorPivotL.set(0.5);
        }

    public void pivotCollectorUp() {
        collectorPivotL.set(-0.5);
    }

    public void manualCollectorPivot() {
        collectorPivotL.set(0);
    }

    public void stopPivotizing() {
        collectorPivotL.set(0);
    }

}