package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class StopCollectAutoCommand extends Command {
    private final CollectorSubsystem collectorSubsystem;
    
    public StopCollectAutoCommand(CollectorSubsystem collector) {
        collectorSubsystem = collector;
        addRequirements(collectorSubsystem);
    }

    public void execute() {
       collectorSubsystem.stopCollection();
    }
}
