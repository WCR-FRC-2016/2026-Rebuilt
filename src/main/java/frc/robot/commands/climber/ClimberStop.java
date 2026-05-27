package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberStop extends Command {
    private final ClimberSubsystem climberSubsystem;


    public ClimberStop(ClimberSubsystem climber) {
        climberSubsystem = climber;
        addRequirements(climberSubsystem);
    }

    public void initialize() {
        climberSubsystem.climberStop();
    }
}
