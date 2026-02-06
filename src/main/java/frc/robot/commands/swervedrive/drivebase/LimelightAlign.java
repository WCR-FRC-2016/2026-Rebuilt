package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.NetworkTables;

public class LimelightAlign extends Command {
    public Translation2d translation;

    private SwerveSubsystem driveBase;
    private double desiredAngle;

    public LimelightAlign(SwerveSubsystem swerve) {
        driveBase = swerve;
        translation = new Translation2d();
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        final boolean tv = NetworkTables.getTv();
        
        //checks if limelight has a target:
        if (tv) {
            final double tx = calcTargetAngle();
            System.out.println(tx);

            //checks if limelight is looking within range of April tag:
            if (tx < -2 || tx > 2) {
                //sets speed, amount of rotation, & rotation direction for the robot:
                double rotateAlign = (tx / 41.0) * -100;
                driveBase.drive(translation, Math.toRadians(rotateAlign), true);
                //locks robot in place if limelight is within range (failsafe):
            } else {
                driveBase.drive(translation, 0, true);
            }
        }
        //rotates the robot until limelight gets a target:
        else {
                driveBase.drive(translation, Math.toRadians(0), true);
        }
        // desiredAngle = driveBase.getHeading().getDegrees() + tx;

        // System.out.println(tx);
    }

    private double calcTargetAngle() {
        double[] targetPos_BotSpace = NetworkTables.getTargetPos_BotSpace();
        if (targetPos_BotSpace == null || targetPos_BotSpace.length != 6) {
            System.out.println("faliure");
            return 0.0;
        }
        double tagX = targetPos_BotSpace[0];
        double tagZ = targetPos_BotSpace[2];
        double tagYaw = Math.toRadians(targetPos_BotSpace[4]);
        System.out.println("tagYaw: " + targetPos_BotSpace[4]);
        final double CENTER_OFFSET_Z = 0.736;
        double centerOffsetRotateZ = CENTER_OFFSET_Z * Math.cos(tagYaw);
        double centerOffsetRotateX = CENTER_OFFSET_Z * Math.sin(tagYaw);
        double targetPosZ = tagZ + centerOffsetRotateZ;
        double targetPosX = tagX + centerOffsetRotateX;
        double targetAngle = Math.atan2(targetPosX, targetPosZ);
        return Math.toDegrees(targetAngle);
    }

    @Override
    public boolean isFinished() {

        if (NetworkTables.getTv() && Math.abs(NetworkTables.getTx()) < 0.1) {
            return true;
        }

        return false;

    }

    // @Override
    // public void end(boolean isFinished==true){
    // SwerveSubsystem.stop();
}
