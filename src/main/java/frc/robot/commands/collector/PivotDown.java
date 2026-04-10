package frc.robot.commands.collector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.CollectorSubsystem;
public class PivotDown extends Command {
    Timer timer = new Timer();
    final double PIVOTTIME = 1.0;
    private final CollectorSubsystem collectorSubsystem;
    
    public PivotDown(CollectorSubsystem collector) {
        collectorSubsystem = collector;
        addRequirements(collectorSubsystem);
    }
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        collectorSubsystem.setPivotDown();
    }
    @Override
    public boolean isFinished() {
        if(timer.get() > PIVOTTIME) {
            return true;
        }
        else {
            return false;
        }
    }
}