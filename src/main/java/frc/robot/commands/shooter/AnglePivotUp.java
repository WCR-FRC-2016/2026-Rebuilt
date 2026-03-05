package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AnglePivotUp extends Command {
     private ShooterSubsystem shooterSubsystem;
    double speed;

    public AnglePivotUp(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.anglePivotUp();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopPivotizing();
    }
}
