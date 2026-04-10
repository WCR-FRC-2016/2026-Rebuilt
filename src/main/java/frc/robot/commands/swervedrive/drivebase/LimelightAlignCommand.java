package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.NetworkTables;

public class LimelightAlignCommand extends Command {

  private double tx;
  private double offsetDistanceX;
  public double offsetDistanceZ;
  
  private final int RED_AIM_APRILTAG = 10;
  private final int BLUE_AIM_APRILTAG = 24;

  private final double DESIRED_DISTANCE_X = 0.0;
  private double DESIRED_DISTANCE_Z = 2.5;
  
  private Translation2d translationZero;
  private Translation2d translation;
  
  private SwerveSubsystem driveBase;
  private ShooterSubsystem shooterSubsystem;
  
  public LimelightAlignCommand(SwerveSubsystem swerve, ShooterSubsystem shooter) {
    driveBase = swerve;
    shooterSubsystem = shooter;
    
    translation = new Translation2d();
    translationZero = new Translation2d();
    addRequirements(driveBase, shooterSubsystem);
  }
  
  @Override
  public void initialize(){
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    int[] filterTag = new int[]{
      (alliance == Alliance.Red) ? RED_AIM_APRILTAG : BLUE_AIM_APRILTAG
    };

    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", filterTag);
    LimelightHelpers.setPriorityTagID("limelight", filterTag[0]);
  }
  
  @Override
  public void execute() {
    clearPositioningState();

    final boolean tv = LimelightHelpers.getTV("limelight");

    if (tv) {
      updatePositioningState();

      // proportional rotation control (smoother)
      double kP_rot = 0.02; // TODO: Validate whether this is
      double rotateAlign = (tx / 41.0f) * -100.0f;//-tx * kP_rot;

      // drive using BOTH translation + rotation

      final boolean isAtPosition = isPositionedCorrectly();
      if (!isAtPosition) {
        driveBase.drive(translationZero, Math.toRadians(rotateAlign), false);
      }
      else {
        driveBase.drive(translationZero, 0, true);
        System.out.println("Alignment Finished!");
      }

    //   if (isAtPosition) {
    //     driveBase.drive(translationZero, 0, true);
    //     System.out.println("finished");
    //   }
    //   else {
    //     driveBase.drive(translationZero, Math.toRadians(rotateAlign), false);
    //   }
    }
    else {
      // search for tag
      driveBase.drive(translationZero, Math.toRadians(20), false);
    }
  }
  
  @Override
  public void end(boolean isInterrupted){
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[0]);
    LimelightHelpers.setPriorityTagID("limelight", -1);
    System.out.println("Alignment Ended");
  }
  
  private void updatePositioningState() {
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

    // NOTE: At the moment this isn't actually adjusting the hood based on the distance,
    //       this is something that needs to be done
    // Use the following length for the calculations:
    final double distanceToTarget = Math.sqrt(targetPosX * targetPosX + targetPosZ * targetPosZ);
    // TODO: Shooter speed based on current position

    // final double desiredRotatedZ =
    //   DESIRED_DISTANCE_Z * Math.cos(targetAngle) -
    //   DESIRED_DISTANCE_X * Math.sin(targetAngle);

    // final double desiredRotatedX =
    //   DESIRED_DISTANCE_Z * Math.sin(targetAngle) +
    //   DESIRED_DISTANCE_X * Math.cos(targetAngle);

    // offsetDistanceX = targetPosX - desiredRotatedX;
    // offsetDistanceZ = targetPosZ - desiredRotatedZ;

    // smoother translation control
    // double kP_pos = 0.8;

    // final double velocityX =
    //   Math.abs(offsetDistanceX) > 0.05 ? offsetDistanceX * kP_pos : 0;

    // final double velocityZ =
    //   Math.abs(offsetDistanceZ) > 0.05 ? offsetDistanceZ * kP_pos : 0;

    // translation = new Translation2d(velocityZ, velocityX);

    System.out.println("Distance to target:" + distanceToTarget);
  }
  
  private void clearPositioningState(){
    offsetDistanceX = 0.0;
    offsetDistanceZ = 0.0;
    tx = 0.0;
  }

  private boolean isPositionedCorrectly(){
    final boolean isAngleInRange = tx > -1 && tx < 1;

    final boolean isAtTargetPosition =
      Math.abs(offsetDistanceX) < 0.1 &&
      Math.abs(offsetDistanceZ) < 0.1;

    return isAngleInRange && isAtTargetPosition;
  }
  
  @Override
  public boolean isFinished() {
    return isPositionedCorrectly();
  }
}