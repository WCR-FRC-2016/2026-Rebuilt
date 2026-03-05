package frc.robot.subsystems.agitator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AgitatorSubsystem extends SubsystemBase {
    private final SparkMax agitatorTread;
    //private static int agitatorTreadCanId = -1;
   // public static double treadPower = -1;

    public AgitatorSubsystem() {
        agitatorTread = null;//new SparkMax(agitatorTreadCanId, MotorType.kBrushless);
    }
 
    public void agitate(double x) {
        
        agitatorTread.set(x);
    }
    public void stopAgitate(){
        agitatorTread.set(0.0);
    }
}
