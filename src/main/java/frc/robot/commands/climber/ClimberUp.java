package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;


public class ClimberUp extends Command {
    Timer timer = new Timer();
    final double PIVOTTIME = 2.1;
    private final ClimberSubsystem climberSubsystem;


    public ClimberUp(ClimberSubsystem climber) {
        climberSubsystem = climber;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        climberSubsystem.runClimberUp();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > PIVOTTIME) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        climberSubsystem.climberStop();
    }
}
