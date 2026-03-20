package frc.robot.subsystems.collector;

import java.nio.BufferOverflowException;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CollectorSubsystem extends SubsystemBase {
    public enum PivotState {
        up, down, manual, shoot
    }

    private enum WheelState {
        collect, spit, off
    }

    private static final int COLLECTOR_WHEELS_L_CAN_ID = 9;
    private static final int COLLECTOR_WHEELS_F_CAN_ID = 8;
    private static final int COLLECTOR_PIVOT_L_CAN_ID = 14;
    private static final int COLLECTOR_PIVOT_F_CAN_ID = 15;
    private static final double COLLECT_POWER = -0.8;
    private static final double PIVOT_DOWN =  -2.282;
    private static final double PIVOT_UP =  0.0;
    private static final double PIVOT_SHOOT =  -1.214;
    

    private final SparkMax collectorWheelsL;
    //private final SparkMax collectorWheelsF;
    public final SparkMax collectorPivotL;
    private final SparkMax collectorPivotF;

    public PivotState desiredPivotState = PivotState.up;
    private WheelState currentWheelState = WheelState.off;

    private DoubleSupplier manualControlInput = null;

    public CollectorSubsystem(DoubleSupplier manualControlInput) {
        
        collectorWheelsL = new SparkMax(COLLECTOR_WHEELS_L_CAN_ID, MotorType.kBrushed);
        //collectorWheelsF = new SparkMax(COLLECTOR_WHEELS_F_CAN_ID, MotorType.kBrushless);
        collectorPivotL = new SparkMax(COLLECTOR_PIVOT_L_CAN_ID, MotorType.kBrushed);
        collectorPivotF = new SparkMax(COLLECTOR_PIVOT_F_CAN_ID, MotorType.kBrushed);

        this.manualControlInput = manualControlInput;

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig().countsPerRevolution(280);
        ClosedLoopConfig pivotClosedLoopConfig = new ClosedLoopConfig().pid(0.85, 0.0, 0.0, ClosedLoopSlot.kSlot0)
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).positionWrappingEnabled(false).outputRange(-1,1);
        SparkMaxConfig collectorPivotLConfig = new SparkMaxConfig();
        SparkMaxConfig collectorPivotFConfig = new SparkMaxConfig();
        SparkMaxConfig collectorWheelsLConfig = new SparkMaxConfig();
        SparkMaxConfig collectorWheelsFConfig = new SparkMaxConfig();

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
       // collectorWheelsF.configure(collectorWheelsFConfig, ResetMode.kResetSafeParameters,
        //        PersistMode.kPersistParameters);

        collectorPivotL.getClosedLoopController().setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

   @Override
public void periodic() {
    final SparkClosedLoopController closedLoopController = collectorPivotL.getClosedLoopController();

    if (desiredPivotState == PivotState.manual) {
        final double currentSetpoint = closedLoopController.getSetpoint();
        double leftJoystickY = manualControlInput.getAsDouble();
        if (leftJoystickY > 0.4 && currentSetpoint < 0) {
            final double newSetpoint = currentSetpoint + (0.05);
            closedLoopController.setSetpoint(newSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
        if (leftJoystickY < -0.4 && currentSetpoint > -2) {
            final double newSetpoint = currentSetpoint - (0.05);
            closedLoopController.setSetpoint(newSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }    
    }

    // System.out.println("setpoint: " + collectorPivotL.getClosedLoopController().getSetpoint() + ", encoder: "
    //         + collectorPivotL.getAlternateEncoder().getPosition());
}
        

       //System.out.println("setpoint: " + collectorPivotL.getClosedLoopController().getSetpoint() + ", encoder: "
          //      + collectorPivotL.getAlternateEncoder().getPosition());

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
            collectorPivotL.getClosedLoopController().setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
        collectorPivotL.getClosedLoopController().setSetpoint(PIVOT_POSITON, ControlType.kPosition,
                ClosedLoopSlot.kSlot0);
    }

    /*
     * public void Collect() {
     * pivotCollectorDown();
     * collectorWheelsL.set(COLLECT_POWER);
     * }
     * 
     * public void stopCollect() {
     * collectorWheelsL.set(0);
     * }
     * 
     * public void Release() {
     * collectorWheelsL.set(-COLLECT_POWER);
     * }
     * 
     * // has to pivot more than 90 degrees
     * public void pivotCollectorDown() {
     * collectorPivotL.set(0.5);
     * }
     * 
     * public void pivotCollectorUp() {
     * collectorPivotL.set(-0.5);
     * }
     * 
     * public void manualCollectorPivot() {
     * collectorPivotL.set(0);
     * }
     * 
     * public void stopPivotizing() {
     * collectorPivotL.set(0);
     * }
     */
}