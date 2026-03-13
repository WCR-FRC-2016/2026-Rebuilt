package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StartShootingAuto80speed extends Command {
    private ShooterSubsystem shooterSubsystem;
    private final double SHOOTERSPEED = 0.8;

    public void StartShootingAuto(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);
    }
@Override

    public void initialize() {
        shooterSubsystem.ShooterWheelsRunAuto(SHOOTERSPEED);
    }
    @Override
    public void end(boolean interrupted) {
    }
}
