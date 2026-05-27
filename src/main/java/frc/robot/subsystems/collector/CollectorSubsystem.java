package frc.robot.subsystems.collector;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorSubsystem extends SubsystemBase {
    public enum PivotState {
        up, down, manual, shoot, bounce
    }

    private enum WheelState {
        collect, spit, off
    }

    private static final int COLLECTOR_WHEELS_L_CAN_ID = 9;
    private static final int COLLECTOR_PIVOT_L_CAN_ID = 14;
    private static final int COLLECTOR_PIVOT_F_CAN_ID = 15;

    private static final double PIVOT_ENCODER_OFFSET = 0.4;

    private static final double COLLECT_POWER = -0.9;
    private static final double PIVOT_DOWN = 0.452;//-0.686;
    private static final double PIVOT_UP = 0.8;//0.003;

    private static final double PIVOT_UP_P = 6.75; // 3.75
    private static final double PIVOT_UP_I = 0.0;
    private static final double PIVOT_UP_D = 0.0;

    private static final double PIVOT_DOWN_P = 3.5; // 1.75
    private static final double PIVOT_DOWN_I = 0.0;
    private static final double PIVOT_DOWN_D = 0.0;

    private static final double PIVOT_BOUNCE_P = 3.75; // TODO: Tune this properly
    private static final double PIVOT_BOUNCE_I = 0.0;
    private static final double PIVOT_BOUNCE_D = 0.0;

    private static final double PIVOT_BOUNCE_CENTER = -0.5f;
    private static final double PIVOT_BOUNCE_AMPLITUDE = 0.186f;

    private static final ClosedLoopSlot PIVOT_UP_SLOT = ClosedLoopSlot.kSlot1;
    private static final ClosedLoopSlot PIVOT_DOWN_SLOT = ClosedLoopSlot.kSlot0;
    private static final ClosedLoopSlot PIVOT_BOUNCE_SLOT = ClosedLoopSlot.kSlot2;

    private final SparkMax collectorWheelsL;
    // private final SparkMax collectorWheelsF;
    public final SparkMax collectorPivotL;
    private final SparkMax collectorPivotF;

    public PivotState desiredPivotState = PivotState.up;
    private WheelState currentWheelState = WheelState.off;

    private DoubleSupplier manualControlInput = null;

    Timer timer = new Timer();

    public CollectorSubsystem(DoubleSupplier manualControlInput) {

        collectorWheelsL = new SparkMax(COLLECTOR_WHEELS_L_CAN_ID, MotorType.kBrushed);
        collectorPivotL = new SparkMax(COLLECTOR_PIVOT_L_CAN_ID, MotorType.kBrushed);
        collectorPivotF = new SparkMax(COLLECTOR_PIVOT_F_CAN_ID, MotorType.kBrushed);

        this.manualControlInput = manualControlInput;

        final SparkMaxConfig globalConfig = new SparkMaxConfig();
        //final AlternateEncoderConfig encoderConfig = AlternateEncoderConfig.Presets.REV_ThroughBoreEncoder;
        final AbsoluteEncoderConfig encoderConfig = AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder;
        final ClosedLoopConfig pivotClosedLoopConfig = new ClosedLoopConfig()
                .pid(PIVOT_DOWN_P, PIVOT_DOWN_I, PIVOT_DOWN_D, PIVOT_DOWN_SLOT) // 0.85 // 1.25 // Pivot Down PIDs
                .pid(PIVOT_UP_P, PIVOT_UP_I, PIVOT_UP_D, PIVOT_UP_SLOT) // 1.0 // 2.0 // Pivot Up PIDs
                .pid(PIVOT_BOUNCE_P, PIVOT_BOUNCE_I, PIVOT_BOUNCE_D, PIVOT_BOUNCE_SLOT) // Pivot Bounce PIDs
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(false)
                .outputRange(-1, 1);
        final SparkMaxConfig collectorPivotLConfig = new SparkMaxConfig();
        final SparkMaxConfig collectorPivotFConfig = new SparkMaxConfig();
        final SparkMaxConfig collectorWheelsLConfig = new SparkMaxConfig();
        final SparkMaxConfig collectorWheelsFConfig = new SparkMaxConfig();

        globalConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);

        collectorPivotLConfig
                .apply(globalConfig)
                .apply(encoderConfig).apply(pivotClosedLoopConfig);
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

        collectorPivotL.getClosedLoopController().setSetpoint(PIVOT_UP, ControlType.kPosition, PIVOT_UP_SLOT);
    }

    @Override
    public void periodic() {
        if (desiredPivotState == PivotState.bounce) {
            double amplitude = 0.125;
            

        }
        final SparkClosedLoopController closedLoopController = collectorPivotL.getClosedLoopController();
        if (desiredPivotState == PivotState.manual) {
            final double currentSetpoint = closedLoopController.getSetpoint();
            double leftJoystickY = manualControlInput.getAsDouble();
            if (leftJoystickY > 0.4 && currentSetpoint < 0) {
                final double newSetpoint = currentSetpoint + (0.05);
                closedLoopController.setSetpoint(newSetpoint, ControlType.kPosition, PIVOT_UP_SLOT);
            }
            if (leftJoystickY < -0.4 && currentSetpoint > -2) {
                final double newSetpoint = currentSetpoint - (0.05);
                closedLoopController.setSetpoint(newSetpoint, ControlType.kPosition, PIVOT_DOWN_SLOT);
            }
        }
    }

    public void startCollecting() {
        currentWheelState = WheelState.collect;
        collectorWheelsL.set(COLLECT_POWER);
    }

    public void startSpitting() {
        currentWheelState = WheelState.spit;
        collectorWheelsL.set(-COLLECT_POWER);
    }

    public void stopCollection() {
        currentWheelState = WheelState.off;
        collectorWheelsL.set(0);
    }

    public void setPivotUp() {
        desiredPivotState = PivotState.up;
        updatePivot();
        System.out.println("Going up");
    }

    public void setPivotDown() {
        desiredPivotState = PivotState.down;
        updatePivot();
        System.out.println("Going down");
    }

    public void setPivotManually() {
        desiredPivotState = PivotState.manual;
        updatePivot();
    }

    public void setPivotShoot() {
        desiredPivotState = PivotState.shoot;
        updatePivot();
    }

    public void zeroPivotEncoder() {
        if (desiredPivotState == PivotState.manual) {
            collectorPivotL.getAlternateEncoder().setPosition(0.0);
            collectorPivotL.getClosedLoopController().setSetpoint(0, ControlType.kPosition, PIVOT_UP_SLOT);
            collectorPivotL.getClosedLoopController();
        } else {
            System.out.println("Cannot reset collector pivot encoder outside of manual mode!");
        }
    }

    public void print() {
        System.out.println("encoder: " + collectorPivotL.getAlternateEncoder().getPosition());
    }

    private void updatePivot() {
        if (desiredPivotState == PivotState.manual)
            return;
        final double PIVOT_POSITON = (desiredPivotState == PivotState.down) ? PIVOT_DOWN : PIVOT_UP;
        final ClosedLoopSlot PIVOT_SLOT = (desiredPivotState == PivotState.down) ? PIVOT_DOWN_SLOT : PIVOT_UP_SLOT;
        collectorPivotL.getClosedLoopController().setSetpoint(PIVOT_POSITON, ControlType.kPosition, PIVOT_SLOT);
    }
}