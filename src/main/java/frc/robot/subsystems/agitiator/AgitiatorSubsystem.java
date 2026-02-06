package frc.robot.subsystems.agitiator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitiatorSubsystem extends SubsystemBase {
    private final SparkMax agitiatorTread;
    private static int agitiatorTreadCanId = -1;
    private static double treadPower = -1;

    public AgitiatorSubsystem() {
        agitiatorTread = new SparkMax(agitiatorTreadCanId, MotorType.kBrushless);
    }

    public void agitiate() {
        agitiatorTread.set(treadPower);
    }
}
