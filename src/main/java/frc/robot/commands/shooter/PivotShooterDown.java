package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PivotShooterDown extends Command {
     private ShooterSubsystem shooterSubsystem;
    double speed;

    public PivotShooterDown(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.pivotShooterDown(speed);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopPivotizing();
    }
}
