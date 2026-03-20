package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PassBalls extends Command {
    private ShooterSubsystem shooterSubsystem;
    private AgitatorSubsystem agitatorSubsystem;

    public PassBalls(ShooterSubsystem shooterSubsystem, AgitatorSubsystem agitatorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.agitatorSubsystem = agitatorSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
       shooterSubsystem.pivotToPass();
    }

    @Override
    public void execute() {
        shooterSubsystem.shooterWheelsRunPass();
        agitatorSubsystem.agitateIfPassSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.pivotToShoot();
        shooterSubsystem.ShooterWheelsStop();
        agitatorSubsystem.stopAgitating();
    }

}
