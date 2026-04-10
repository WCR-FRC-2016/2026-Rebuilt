package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import edu.wpi.first.wpilibj.Timer;

// TODO: Review this class (and potentially remove it)
public class ClimbAutoCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    Timer timer = new Timer();
    final double CLIMBTIME = 1.3;

    public ClimbAutoCommand(ClimberSubsystem climber) {
        climberSubsystem = climber;
        addRequirements(climberSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        climberSubsystem.runClimberDown();
    }

    public void execute() {
        if (timer.get() > CLIMBTIME) {
            climberSubsystem.climberStop();
        }
    }
}
