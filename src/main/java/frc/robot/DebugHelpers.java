package frc.robot;
import edu.wpi.first.wpilibj.DigitalInput;

public class DebugHelpers {
    public static void printLimelightTx(String label)
    {
        System.out.print(label);
        System.out.println(LimelightHelpers.getTX("limelight"));
    }

    public static void printLimelightTy(String label)
    {
        System.out.print(label);
        System.out.println(LimelightHelpers.getTY("limelight"));
    }

    public static void printLimelightTv(String label)
    {
        System.out.print(label);
        System.out.println(LimelightHelpers.getTV("limelight"));
    }
    public static void printMagneticLimitSwitchValue(String label, DigitalInput magneticLimitSwitch)
    {
        System.out.print(label);
        System.out.println(magneticLimitSwitch.get());
    }
}
