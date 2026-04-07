package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.NetworkTables;

public class LimelightPowerAlignCommand extends Command {
    private record Datapoint(double Distance, double flyWheelSpeed) {}
    private double tx;
    private double flyWheelSpeed = 0.0;
    public double offsetDistanceZ;


    private static final Datapoint[] data = {
            // sorted by distance increasing order (distance of 1 would go here)
            // (TooClose,0.0)
           
            // (TooFar, -1.58)
    };
    // private double desiredDistance;

    // 2.1;//speed 60
    // 3 ?
    // 4.1 ?
    // 5.3 ?
    // 6.3 ?

    private Translation2d translationZero;
    private SwerveSubsystem driveBase;
    private ShooterSubsystem shooterSubsystem;

    public LimelightPowerAlignCommand(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        driveBase = swerve; 
        shooterSubsystem = shooter;

        new Translation2d();
        translationZero = new Translation2d();
       // addRequirements(driveBase,shooterSubsystem);

    }

    @Override
    public void initialize() {
        int[] filterTag = new int[] {
               // (DriverStation.getAlliance().get() == Alliance.Red) ? RED_AIM_APRILTAG : BLUE_AIM_APRILTAG
        };
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", filterTag);
        LimelightHelpers.setPriorityTagID("limelight", filterTag[0]);

    }

    @Override
    public void execute() {
        clearPositioningState();
        // final boolean tv = NetworkTables.getTv();
  final boolean tv = LimelightHelpers.getTV("limelight");
        if (tv) {
            updatePositioningState();
            //System.out.println("Hood angle: " + flyWheelSpeed);
           shooterSubsystem.DESIRED_SHOOTING_VELOCITY = flyWheelSpeed;
        }
       
        }
     

    @Override
    public void end(boolean isInterrupted) {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[0]);
        LimelightHelpers.setPriorityTagID("limelight", -1);
    }

    @Override
    public boolean isFinished() {
        final boolean tv = LimelightHelpers.getTV("limelight");
        clearPositioningState();

        if (tv) {
            updatePositioningState();
            final boolean isAtPosition = isPositionedCorrectly();
            if (isAtPosition) {
                System.out.println("At position");
            }
            return isAtPosition;
        }
        return false;
    }

    // Target Distance For Shoot : 2.5 m
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
        
        final double distanceToTarget =  Math.sqrt(targetPosX * targetPosX + targetPosZ * targetPosZ);
       double  desiredFlyWheelSpeed = calculateflyWheelSpeedLinear(distanceToTarget);
        //System.out.println("distance: " + distanceToTarget);
        // distance formula
        //System.out.println("length:" + Math.sqrt(targetPosX * targetPosX + targetPosZ * targetPosZ));
    }

    private double calculateflyWheelSpeedLinear( final double distanceToTarget){
        for(int i=0; i< data.length; i++){
            final Datapoint currentDatapoint = data[i];
            if(distanceToTarget > currentDatapoint.Distance){
                continue;
         }
            if(i<=0){
                return currentDatapoint.flyWheelSpeed;
            }
            final Datapoint previousDatapoint = data[i-1];
            //misspelled did not want to mess with it mid tourney
            final double datapoiontDistance = currentDatapoint.Distance - previousDatapoint.Distance;
            final double targetDistanceOffset = distanceToTarget - previousDatapoint.Distance;
            final double t = targetDistanceOffset / datapoiontDistance;
            final  double flyWheelSpeedDifference = currentDatapoint.flyWheelSpeed - previousDatapoint.flyWheelSpeed;
            final double targetAngle = previousDatapoint.flyWheelSpeed + flyWheelSpeedDifference * t;
            return targetAngle;
        }

        System.out.println("Failed to find datapoint, get closer to target");
        return data[data.length - 1].flyWheelSpeed;
    }

    private void clearPositioningState() {
        flyWheelSpeed = 0.0;
        offsetDistanceZ = 0.0;
        tx = 0.0;
    }

    private boolean isPositionedCorrectly() {
        final boolean isAngleInRange = tx > -1 && tx < 1;
        return isAngleInRange;
    }
}
