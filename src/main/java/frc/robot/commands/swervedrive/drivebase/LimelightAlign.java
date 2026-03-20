package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.NetworkTables;

public class LimelightAlign extends Command {
    private double tx;
    private double offsetDistanceX;
    public double offsetDistanceZ;

    private final int RED_AIM_APRILTAG = 10;
    private final int BLUE_AIM_APRILTAG = 24;

    private final double DESIRED_DISTANCE_X = 0.0;
    private  double DESIRED_DISTANCE_Z =2.5;// 60?
   // private double desiredDistance;
    

    // 2.1;//speed 60
    // 3 ?
    // 4.1 ?
    // 5.3 ?
    // 6.3 ?

    private Translation2d translationZero;
    private Translation2d translation;

    private SwerveSubsystem driveBase;
    private ShooterSubsystem shooterSubsystem;

    public LimelightAlign(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        driveBase = swerve;
        shooterSubsystem =shooter;
        
        translation = new Translation2d();
        translationZero = new Translation2d();
        addRequirements(driveBase, shooterSubsystem);
        
    }

     @Override
     public void initialize(){
        int[]filterTag = new int[]{
            (DriverStation.getAlliance().get() == Alliance.Red) ? RED_AIM_APRILTAG : BLUE_AIM_APRILTAG
        };
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight",filterTag);
        LimelightHelpers.setPriorityTagID("limelight", filterTag[0]);

     }
    @Override
    public void execute() {
        clearPositioningState();
        //final boolean tv = NetworkTables.getTv();

        // checks if limelight has a target:
        final boolean tv = LimelightHelpers.getTV("limelight");
        if (tv) {
            updatePositioningState();
        System.out.println("distance");  
            // Check to see if the robot is positioned and rotated proprerly
            final boolean isAtPosition = isPositionedCorrectly();
            if (!isAtPosition) {
                // sets speed, amount of rotation, & rotation direction for the robot:
                double rotateAlign = (tx / 41.0) * -100;
                driveBase.drive(translation, Math.toRadians(rotateAlign), false);
            } else {
                 // locks robot in place if limelight is within range (failsafe):
                driveBase.drive(translationZero, 0, true);
                System.out.println("finished");
            }
        }
        // rotates the robot until limelight gets a target:
        else {
        driveBase.drive(translationZero, Math.toRadians(30), true);
        }
    }
        @Override
        public void end(boolean isInterrupted){
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight",new int [0]);
            LimelightHelpers.setPriorityTagID("limelight", -1);
            System.out.println("Alignment Ended");
        }
    // Target Distance For Shoot : 2.5 m
   
    private void updatePositioningState( ) {
        double[] targetPos_BotSpace = NetworkTables.getTargetPos_BotSpace();
        if (targetPos_BotSpace == null || targetPos_BotSpace.length != 6) {
            System.out.println("faliure");
            return;
        }
        
        final double tagX = targetPos_BotSpace[0];
        final double tagZ = targetPos_BotSpace[2];
        final double tagYaw = Math.toRadians(targetPos_BotSpace[4]);
        final double CENTER_OFFSET_Z = 0.736;
        final double centerOffsetRotateZ = CENTER_OFFSET_Z * Math.cos(tagYaw);
        final double centerOffsetRotateX = CENTER_OFFSET_Z * Math.sin(tagYaw);
        final double targetPosZ = tagZ + centerOffsetRotateZ;
        final double targetPosX = tagX + centerOffsetRotateX;
        final double targetAngle = Math.atan2(targetPosX, targetPosZ);
        tx = Math.toDegrees(targetAngle);

        final double desiredRotatedZ = DESIRED_DISTANCE_Z * Math.cos(targetAngle) - DESIRED_DISTANCE_X * Math.sin(targetAngle);
        final double desiredRotatedX = DESIRED_DISTANCE_Z * Math.sin(targetAngle) + DESIRED_DISTANCE_X * Math.cos(targetAngle);
        offsetDistanceX = targetPosX - desiredRotatedX;
        offsetDistanceZ = targetPosZ - desiredRotatedZ;
        final double velocityX = Math.min(Math.max(offsetDistanceX / 1.5, -2), 2);
        final double velocityZ = Math.min(Math.max(offsetDistanceZ / 1.5, -2), 2);
        translation = new Translation2d(velocityZ, velocityX);
        
        // distance formula
        System.out.println("length:" + Math.sqrt(targetPosX * targetPosX + targetPosZ * targetPosZ));
    }
    
    private void clearPositioningState(){
        offsetDistanceX = 0.0;
        offsetDistanceZ = 0.0;
        tx = 0.0;
    }
    private boolean isPositionedCorrectly(){
        final boolean isAngleInRange = tx > -1 && tx < 1 ; 
        final boolean isAtTargetPosition = Math.abs(offsetDistanceX) < 0.1 && Math.abs(offsetDistanceZ) < 0.1;
        return isAngleInRange && isAtTargetPosition;
    }

    @Override
    public boolean isFinished() {
        final boolean tv = LimelightHelpers.getTV("limelight");
        clearPositioningState();

        if(tv){
            updatePositioningState();
            final boolean isAtPosition = isPositionedCorrectly();
            if(isAtPosition){
            System.out.println("At position");
            }
            return isAtPosition;
        }

         return false;
    }
}
