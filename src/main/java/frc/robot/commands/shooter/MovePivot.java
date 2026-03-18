package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class MovePivot extends Command {
    private ShooterSubsystem shooterSubsystem;
    double position;

    public MovePivot(ShooterSubsystem shooter, double position) {
        this.shooterSubsystem = shooter;
        this.position = position;
        addRequirements(shooterSubsystem);
    }


    @Override

    public void initialize() {
    
    }

    @Override
    public void execute() {
        shooterSubsystem.pivotTo(position);
       // System.out.println("Uh oh stinky");
    }   

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.pivotTo(0);
    }

}
