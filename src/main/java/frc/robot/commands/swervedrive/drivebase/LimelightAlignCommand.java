package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.NetworkTables;
import edu.wpi.first.math.Vector;

public class LimelightAlignCommand extends Command {

  /*   private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private Vector<N2> position;
    Translation2d translationZero;
    double offsetRotation;


    public LimelightAlignCommand(final SwerveSubsystem swerveSubsystem, final ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        translationZero = new Translation2d();
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        updatePositioningState();
        final boolean tv = LimelightHelpers.getTV("limelight");
        if(tv){
             //Check to see if the robot is positioned and rotated proprerly
            final boolean isAtPosition = isPositionedCorrectly();
           if (!isAtPosition) {
               // sets speed, amount of rotation, & rotation direction for the robot:
                swerveSubsystem.drive(translationZero, Math.toRadians(offsetRotation*2), false);
            } else {
               // locks robot in place if limelight is within range (failsafe):
               swerveSubsystem.drive(translationZero, 0, true);
               System.out.println("finished");
            }
        }
        }
    

    @Override
    public boolean isFinished() {
        return false;
    }

    private void updatePositioningState() {
        LimelightHelpers.PoseEstimate position = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (position == null) {
            System.out.println("faliure");
            return;
        }

        System.out.println("x: " + position.pose.getX() + ", y: " + position.pose.getY() + ", heading: "
                + position.pose.getRotation().getDegrees());

        double botPoseX = position.pose.getX();
        double botPoseY = position.pose.getY();
        double botPoseHeading = position.pose.getRotation().getDegrees();
        double distanceFromHub = 0;
        double distanceFromHubAprilTag = 0;
        double aprilTagDistanceToHubCenter = 0;

        Alliance allience = Alliance.Red;

        double targetX = 0;
        double targetY = 4.02;

        if (allience == Alliance.Red) {
            targetX = 11.9;
            aprilTagDistanceToHubCenter = -0.55;
        } else if (allience == Alliance.Blue) {
            targetX = 4.612;
            aprilTagDistanceToHubCenter = -0.55;
        }
        
        double centerDistanceX = aprilTagDistanceToHubCenter + targetX;
        distanceFromHub = Math.sqrt(Math.pow(botPoseX - targetX, 2) + Math.pow(botPoseY - targetY, 2));
        distanceFromHubAprilTag = Math.sqrt(Math.pow(botPoseX - targetX + aprilTagDistanceToHubCenter, 2) + Math.pow(botPoseY - 4.02, 2));
        
        double dx = targetX - botPoseX;
        double dy = targetY - botPoseY;

        double targetAngle = Math.atan2(dy, dx);
        double currentHeadingRad = Math.toRadians(botPoseHeading);

        double desiredAngleChange = targetAngle - currentHeadingRad;
        // desired angle change needs to be more
        
        double desiredAngleChangeDegrees = Math.toDegrees(desiredAngleChange);
        offsetRotation = desiredAngleChangeDegrees;

        // desiredAngleChange = Math.atan2(Math.sin(desiredAngleChange), Math.cos(desiredAngleChange));

        //System.out.println("heading : " + botPoseHeading);
      //  System.out.println("desiredAngleChange : " + desiredAngleChangeDegrees);
        //System.out.println("distanceFromHub : " + distanceFromHub);
        //System.out.println("botPoseX: " + botPoseX);
        //System.out.println("botPoseY: " + botPoseY);
    }
    
    private boolean isPositionedCorrectly() {
        final boolean isAngleInRange = offsetRotation > -2 && offsetRotation < 2;
        return isAngleInRange;
    }
}   
*/

    
      private double tx;
      private double offsetDistanceX;
      public double offsetDistanceZ;
      
      private final int RED_AIM_APRILTAG = 10;
      private final int BLUE_AIM_APRILTAG = 24;
    
      private final double DESIRED_DISTANCE_X = 0.0;
      private double DESIRED_DISTANCE_Z =2.5;// 60?
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
      
      public LimelightAlignCommand(SwerveSubsystem swerve, ShooterSubsystem
      shooter) {
      driveBase = swerve;
      shooterSubsystem =shooter;
      
      translation = new Translation2d();
      translationZero = new Translation2d();
      addRequirements(driveBase, shooterSubsystem);
      
      }
      
      @Override
      public void initialize(){
      int[]filterTag = new int[]{
      (DriverStation.getAlliance().get() == Alliance.Red) ? RED_AIM_APRILTAG :
      BLUE_AIM_APRILTAG
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
      driveBase.drive(translationZero, Math.toRadians(rotateAlign), false);
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
      
      final double desiredRotatedZ = DESIRED_DISTANCE_Z * Math.cos(targetAngle) -
      DESIRED_DISTANCE_X * Math.sin(targetAngle);
      final double desiredRotatedX = DESIRED_DISTANCE_Z * Math.sin(targetAngle) +
      DESIRED_DISTANCE_X * Math.cos(targetAngle);
      offsetDistanceX = targetPosX - desiredRotatedX;
      offsetDistanceZ = targetPosZ - desiredRotatedZ;
      final double velocityX = Math.min(Math.max(offsetDistanceX / 1.5, -2), 2);
      final double velocityZ = Math.min(Math.max(offsetDistanceZ / 1.5, -2), 2);
      translation = new Translation2d(velocityZ, velocityX);
      
      // distance formula
      System.out.println("length:" + Math.sqrt(targetPosX * targetPosX + targetPosZ
      * targetPosZ));
      }
      
      private void clearPositioningState(){
      offsetDistanceX = 0.0;
      offsetDistanceZ = 0.0;
      tx = 0.0;
      }
      private boolean isPositionedCorrectly(){
      final boolean isAngleInRange = tx > -1 && tx < 1 ;
      final boolean isAtTargetPosition = Math.abs(offsetDistanceX) < 0.1 &&
      Math.abs(offsetDistanceZ) < 0.1;
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
