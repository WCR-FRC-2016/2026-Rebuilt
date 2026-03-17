package frc.robot.commands.collector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;

public class movePivotDown extends Command {
    private final CollectorSubsystem collectorSubsystem;

    public movePivotDown(CollectorSubsystem collector) {
        collectorSubsystem = collector;
        addRequirements(collectorSubsystem);
    }

    @Override
    public void initialize() {
        collectorSubsystem.setPivotDown();
    }

   
    @Override
    public boolean isFinished() {
        if(collectorSubsystem.desiredPivotState == CollectorSubsystem.PivotState.down) {
            return true;
        }
        return false;
    }
}
