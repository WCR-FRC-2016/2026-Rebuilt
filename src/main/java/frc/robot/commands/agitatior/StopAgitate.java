package frc.robot.commands.agitatior;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;

public class StopAgitate extends Command 
{
    private final AgitatorSubsystem agitatorSubsystem;

    public StopAgitate(AgitatorSubsystem agitator) {
        agitatorSubsystem = agitator;
        addRequirements(agitatorSubsystem);
    }

    @Override
    public void initialize() {
        agitatorSubsystem.stopAgitating();
    }
    @Override
    public void end(boolean interrupted) {
       
    }
    
}
