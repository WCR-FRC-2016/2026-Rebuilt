package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private static final int CLIMBER_MOTOR_CAN_ID = 59;

    @SuppressWarnings("removal")
    public ClimberSubsystem() {
        climberMotor = new SparkMax(CLIMBER_MOTOR_CAN_ID, MotorType.kBrushed);
        motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        climberMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runClimber() {
        climberMotor.set(Constants.SpeedConstants.CLIMBER_SPEED);
    }

    public void stopClimbing() {
        climberMotor.stopMotor();
    }
}
