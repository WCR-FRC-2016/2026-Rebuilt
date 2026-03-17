package frc.robot.commands.swervedrive.agitator;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootAgitate extends Command {
    private Agitate agitatorCommand;
    public ShootAgitate(Agitate agitatorCommand) {
        this.agitatorCommand = agitatorCommand;
        addRequirements(agitatorCommand.getRequirements().toArray(new edu.wpi.first.wpilibj2.command.Subsystem[0]));
    }

    @Override
    public void execute() {
        agitatorCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        agitatorCommand.end(interrupted);
    }
    
}
