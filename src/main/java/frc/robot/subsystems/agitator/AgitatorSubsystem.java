package frc.robot.subsystems.agitator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase {
    
  private static final int AGITATOR_TREAD_CAN_ID = 2;
    private final SparkMax agitatorTread;
    public final double AGITATESPEED = 0.75;

    @SuppressWarnings("removal")
    public AgitatorSubsystem() {
        agitatorTread = new SparkMax(AGITATOR_TREAD_CAN_ID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        agitatorTread.configure(config, SparkMax.ResetMode.kResetSafeParameters, 
                                SparkMax.PersistMode.kPersistParameters);
    }

    public void startAgitating() {
        agitatorTread.set(AGITATESPEED);
    }

    public void stopAgitating() {
        agitatorTread.set(0.0);
    }
}