package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StopShootingAuto extends Command {
    private ShooterSubsystem shooterSubsystem;

    public void StartShootingAuto(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);
    }
    @Override

    public void initialize() {
        shooterSubsystem.ShooterWheelsStop();;
    }
    @Override
    public void end(boolean interrupted) {
    }

}
