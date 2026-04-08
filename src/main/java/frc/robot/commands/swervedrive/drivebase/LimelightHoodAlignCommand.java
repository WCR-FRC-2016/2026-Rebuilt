package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.NetworkTables;

public class LimelightHoodAlignCommand extends Command {
    private record Datapoint(double Distance, double HoodAngle) {
    }

    private double hoodPivotAngle = 0.0;
    public double offsetDistanceZ;

    private final int RED_AIM_APRILTAG = 10;
    private final int BLUE_AIM_APRILTAG = 24;

    private static final Datapoint[] data = {
            // sorted by distance increasing order (distance of 1 would go here)
            // (TooClose,0.0)
            new Datapoint(2.005, 0.035),
            new Datapoint(2.257, -0.371),
            new Datapoint(2.713, -0.861),
            // (TooFar, -1.58)
    };

    // 2.1;//speed 60
    // 3 ?
    // 4.1 ?
    // 5.3 ?
    // 6.3 ?

    private ShooterSubsystem shooterSubsystem;

    public LimelightHoodAlignCommand(SwerveSubsystem swerve, ShooterSubsystem shooter) {
        shooterSubsystem = shooter;

        new Translation2d();
        new Translation2d();

    }

    @Override
    public void initialize() {
        int[] filterTag = new int[] {
                (DriverStation.getAlliance().get() == Alliance.Red) ? RED_AIM_APRILTAG : BLUE_AIM_APRILTAG
        };
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", filterTag);
        LimelightHelpers.setPriorityTagID("limelight", filterTag[0]);

    }

    @Override
    public void execute() {
        clearPositioningState();
        // final boolean tv = NetworkTables.getTv();

        // checks if limelight has a target:
        final boolean tv = LimelightHelpers.getTV("limelight");
        if (tv) {
            updatePositioningState();
            //System.out.println("Hood angle: " + hoodPivotAngle);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", new int[0]);
        LimelightHelpers.setPriorityTagID("limelight", -1);
    }

    @Override
    public boolean isFinished() {
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
        Math.toDegrees(targetAngle);

        final double distanceToTarget = Math.sqrt(targetPosX * targetPosX + targetPosZ * targetPosZ);
        hoodPivotAngle = calculateHoodAngleLinear(distanceToTarget);
       // System.out.println("distance: " + distanceToTarget);
    }

    private double calculateHoodAngleLinear(final double distanceToTarget) {
        for (int i = 0; i < data.length; i++) {
            final Datapoint currentDatapoint = data[i];
            if (distanceToTarget > currentDatapoint.Distance) {
                continue;
            }
            if (i <= 0) {
                return currentDatapoint.HoodAngle;
            }
            final Datapoint previousDatapoint = data[i - 1];
            // misspelled did not want to mess with it mid tourney
            final double datapoiontDistance = currentDatapoint.Distance - previousDatapoint.Distance;
            final double targetDistanceOffset = distanceToTarget - previousDatapoint.Distance;
            final double t = targetDistanceOffset / datapoiontDistance;
            final double hoodAngleDifference = currentDatapoint.HoodAngle - previousDatapoint.HoodAngle;
            final double targetAngle = previousDatapoint.HoodAngle + hoodAngleDifference * t;
            return targetAngle;
        }

        System.out.println("Failed to find datapoint, get closer to target");
        return data[data.length - 1].HoodAngle;
    }

    private void clearPositioningState() {
        hoodPivotAngle = 0.0;
        offsetDistanceZ = 0.0;
    }
}
