package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StartShootingAuto30speed extends Command {
    private ShooterSubsystem shooterSubsystem;
    private final double SHOOTERSPEED = 0.3;

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
