package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTables {

    public static double[] getBotPos(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[0]);
    }

    public static double[] getTargetPos_BotSpace(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
    }
    
    public static double getTx(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }


    public static boolean getTv() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) > 0;
    }

    public static double getAprilTagID(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0.0);
    }


    
}