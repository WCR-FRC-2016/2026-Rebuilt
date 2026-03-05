package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterWheelsRun extends Command {
    ShooterSubsystem shooterSubsystem;
    double speed;

    public ShooterWheelsRun(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        shooterSubsystem.ShooterWheelsRun(speed);
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.ShooterWheelsStop();
    }
}