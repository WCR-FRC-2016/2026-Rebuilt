package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

// TODO: Review this class (and potentially remove it)
public class StartCollectAutoCommand extends Command {
    private final CollectorSubsystem collectorSubsystem;
    
    public StartCollectAutoCommand(CollectorSubsystem collector) {
        collectorSubsystem = collector;
        addRequirements(collectorSubsystem);
    }

    public void initialize() {
       collectorSubsystem.startCollecting();
    }
}
