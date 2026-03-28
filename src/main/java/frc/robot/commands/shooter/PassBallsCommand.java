package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PassBallsCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private AgitatorSubsystem agitatorSubsystem;

    public PassBallsCommand(ShooterSubsystem shooterSubsystem, AgitatorSubsystem agitatorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.agitatorSubsystem = agitatorSubsystem;
        
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
       shooterSubsystem.setPivotToPass();
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterWheelsPass();
        agitatorSubsystem.agitateIfPassSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setPivotToShoot();
        shooterSubsystem.stopShooterWheels();
        agitatorSubsystem.stopAgitating();
    }
}
