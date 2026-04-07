package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class StartWaitStopCollectionAutoCommand extends Command {
    private final CollectorSubsystem collectorSubsystem;
    Timer timer = new Timer();
    final double COLLECTTIME = 2.0;
    
    public StartWaitStopCollectionAutoCommand(CollectorSubsystem collector) {
        collectorSubsystem = collector;
        addRequirements(collectorSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        collectorSubsystem.startCollecting();
    }
    public void execute() {
        if(timer.get() > COLLECTTIME) {
            collectorSubsystem.stopCollection();
        }
    }
}
