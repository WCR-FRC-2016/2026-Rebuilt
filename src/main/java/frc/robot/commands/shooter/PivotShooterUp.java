package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class PivotShooterUp extends Command {
    private ShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();
    final double PIVOTTIME = 0.5;

    public void StartShootingAuto(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);
    }
    @Override

    public void initialize() {
        timer.reset();
        timer.start();
        shooterSubsystem.pivotUp();
    }

    public void execute() {
        if (timer.get() > PIVOTTIME) {
            shooterSubsystem.stopPivotizing();
        }
    }
        @Override
    public void end(boolean interrupted) {
    }

}
