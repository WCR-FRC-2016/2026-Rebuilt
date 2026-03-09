package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class ClimberSubsystem extends SubsystemBase {

    private static final int CLIMBER_MOTOR_CAN_ID = 6;

    private final SparkMax climberMotor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();

    private Timer climberTimer = new Timer();
    private boolean climberRunning = false;

    @SuppressWarnings("removal")
    public ClimberSubsystem() {

        climberMotor = new SparkMax(CLIMBER_MOTOR_CAN_ID, MotorType.kBrushed);

        motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        climberMotor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void runClimber() {

        if (!climberRunning) {
            climberTimer.reset();
            climberTimer.start();
            climberRunning = true;
        }

        if (climberTimer.get() < 5.0) {
            climberMotor.set(0.5);
        } else {
            climberMotor.stopMotor();
            climberRunning = false;
        }
    }

    public void stop() {
        climberMotor.set(0);
    }
}