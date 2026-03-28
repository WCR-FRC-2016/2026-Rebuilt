package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.NetworkTables;

public class calculateAndMovePivot extends Command {
    private ShooterSubsystem shooterSubsystem;

    double tz;
    double pivotAngle;

    public calculateAndMovePivot(ShooterSubsystem shooter) {
        this.shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        double[] targetPos_BotSpace = NetworkTables.getTargetPos_BotSpace();

        if (targetPos_BotSpace == null || targetPos_BotSpace.length != 6) {
            return;
        }

        double tagX = targetPos_BotSpace[0];
        double tagZ = targetPos_BotSpace[2];

        tz = Math.sqrt((tagX * tagX) + (tagZ * tagZ));
        
        pivotAngle = tz + 1; // your formula here

        //shooterSubsystem.pivotTo(pivotAngle);
        shooterSubsystem.pivotToPosition(0);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.pivotToPosition(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}