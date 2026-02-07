package frc.robot;

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
}
