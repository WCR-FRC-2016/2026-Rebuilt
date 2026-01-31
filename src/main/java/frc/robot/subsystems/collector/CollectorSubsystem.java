package frc.robot.subsystems.collector;

public class CollectorSubsystem {
    
}
public class CollectorSubsystem extends SubsystemBase {
private final SparkMax collectorWheels;
private final SparkMax collectorPivot;
private static int collectorWheelsCanId = -1;
private static int collectorPivotCanID = -1;
private static double collectPower = -1;
private static double uncollectPower = -1;
PIDController pid = new PIDController (0, 0, 0);
public CollectorSubsystem(){
    collectorWheels = new SparkMax(collectorWheelsCanId,MotorType.kBrushed);
    collectorPivot = new SparkMax(collectorPivotCanId,MotorType.kBrushless);
}
public void collect(){
    collectorWheels.set(collectPower);
}

public void uncollect(){
    collectorWheels.set(uncolllectPower);
}

public void collectorPivot(){
    
}
}