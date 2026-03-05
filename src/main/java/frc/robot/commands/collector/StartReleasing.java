package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StartReleasing extends Command {
     private CollectorSubsystem collectorSubsystem;
    double speed;

    public StartReleasing(CollectorSubsystem collectorSubsystem, double speed){
        this.collectorSubsystem = collectorSubsystem;
        this.speed = speed;
        addRequirements(collectorSubsystem);
    }

    @Override
    public void execute(){
        collectorSubsystem.startReleasing();
    }

    @Override
    public void end(boolean interrupted){
        collectorSubsystem.stopCollecting();
    }
}
