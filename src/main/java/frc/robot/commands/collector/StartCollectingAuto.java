package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class StartCollectingAuto extends Command{
    private final CollectorSubsystem collectorSubsystem;

    public StartCollectingAuto(CollectorSubsystem collector) {
        collectorSubsystem = collector;
        addRequirements(collectorSubsystem);
    }

    public void initialize() {
        collectorSubsystem.pivotCollectorDown();
        collectorSubsystem.Collect();
    }
}
