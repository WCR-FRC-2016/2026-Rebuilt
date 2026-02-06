package frc.robot.subsystems.collector;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSubsystem extends SubsystemBase {
    private final SparkMax collectorWheels;
    private final SparkMax collectorPivotM;
    private final SparkMax collectorPivotS;
    private static int collectorWheelsCanId = -1;
    private static int collectorPivotMCanId = -1;
    private static int collectorPivotSCanId = -1;
    private static double collectPower = -1;
    private static double uncollectPower = -1;

    PIDController pid = new PIDController(0, 0, 0);

    public CollectorSubsystem() {
        collectorWheels = new SparkMax(collectorWheelsCanId, MotorType.kBrushed);
        collectorPivotM = new SparkMax(collectorPivotMCanId, MotorType.kBrushless);
        collectorPivotS = new SparkMax(collectorPivotSCanId, MotorType.kBrushless);

        
          SparkMaxConfig globalConfig = new SparkMaxConfig();
          SparkMaxConfig collectorPivotMConfig = new SparkMaxConfig();
          SparkMaxConfig collectorPivotSConfig = new SparkMaxConfig();
          
          
          globalConfig
          .smartCurrentLimit(50)
          .idleMode(IdleMode.kBrake);
          
          collectorPivotMConfig
          .apply(globalConfig)
          .inverted(true);
          
          collectorPivotSConfig
          .apply(globalConfig)
          .follow(collectorPivotM);
          
          collectorPivotM.configure(collectorPivotMConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
          collectorPivotS.configure(collectorPivotSConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
         
    }

    public void collect() {
        collectorWheels.set(collectPower);
    }

    public void uncollect() {
        collectorWheels.set(uncollectPower);
    }

    public void collectorPivot() {
        // has to pivot more than 90 degrees

    }
}
