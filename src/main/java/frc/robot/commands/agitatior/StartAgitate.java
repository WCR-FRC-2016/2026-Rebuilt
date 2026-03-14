package frc.robot.commands.agitatior;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;

public class StartAgitate extends Command 
{
    private final AgitatorSubsystem agitatorSubsystem;

    public StartAgitate(AgitatorSubsystem agitator) {
        agitatorSubsystem = agitator;
        addRequirements(agitatorSubsystem);
    }

    @Override
    public void initialize() {
        agitatorSubsystem.startAgitating();
    }
    @Override
    public void end(boolean interrupted) {
       
    }
    
}
