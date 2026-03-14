package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.NetworkTables;

public class LimelightAlign extends Command {
    private double tx = 0.0;
    private double offsetDistanceX = 0.0;
    private double offsetDistanceZ = 0.0;

    private final double DESIRED_DISTANCE_X = 0.0;
    private final double DESIRED_DISTANCE_Z = 2.5;

    private double desiredAngle;

    private Translation2d translationZero;
    private Translation2d translation;

    private SwerveSubsystem driveBase;

    public LimelightAlign(SwerveSubsystem swerve) {
        driveBase = swerve;
        translation = new Translation2d();
        translationZero = new Translation2d();
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        offsetDistanceX = 0.0;
        offsetDistanceZ = 0.0;
        tx = 0.0;

        final boolean tv = NetworkTables.getTv();

        // checks if limelight has a target:
        if (tv) {
            updatePositioningState();
            System.out.println("tx: " + tx + ", offsetX: " + offsetDistanceX + ", offsetZ: " + offsetDistanceZ);
            // System.out.println(tx);
            
            
            // checks if limelight is looking within range of April tag:
            final boolean isAngleInRange = tx > -1 && tx < 1 ; 
            final boolean isAtTargetPosition = Math.abs(offsetDistanceX) < 0.1 && Math.abs(offsetDistanceZ) < 0.1;
            if (!isAngleInRange || !isAtTargetPosition) {
                // sets speed, amount of rotation, & rotation direction for the robot:
                double rotateAlign = (tx / 41.0) * -100;
                // Translation2d translation = new Translation2d(movementVelocityZ,
                // movementVelocityX);
                driveBase.drive(translation, Math.toRadians(rotateAlign), false);
                // locks robot in place if limelight is within range (failsafe):
                
            } else {
                driveBase.drive(translationZero, 0, true);
                System.out.println("finished");
            }
        }
        // rotates the robot until limelight gets a target:
        else {
            driveBase.drive(translationZero, Math.toRadians(30), true);
        }
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

        final double desiredRotatedZ = DESIRED_DISTANCE_Z * Math.cos(targetAngle) - DESIRED_DISTANCE_X * Math.sin(targetAngle);
        final double desiredRotatedX = DESIRED_DISTANCE_Z * Math.sin(targetAngle) + DESIRED_DISTANCE_X * Math.cos(targetAngle);
        offsetDistanceX = targetPosX - desiredRotatedX;
        offsetDistanceZ = targetPosZ - desiredRotatedZ;
        final double velocityX = Math.min(Math.max(offsetDistanceX / 1.5, -2), 2);
        final double velocityZ = Math.min(Math.max(offsetDistanceZ / 1.5, -2), 2);
        translation = new Translation2d(velocityZ, velocityX);
        //if (translation.getDistance(translationZero))
        //System.out.println("length:" + Math.sqrt(targetPosX * targetPosX + targetPosZ * targetPosZ));

        System.out.println("x: " + translation.getX() + ", Y: " + translation.getY());
    }

    @Override
    public boolean isFinished() {

       /*  if (NetworkTables.getTv() && Math.abs(NetworkTables.getTx()) < 0.1) {
            return true;
        }   */

        return false;

    }

    // @Override
    // public void end(boolean isFinished==true){
    // SwerveSubsystem.stop();
}
