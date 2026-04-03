package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final AgitatorSubsystem agitatorSubsystem;
    Timer timer = new Timer();
    final double SHOOTTIME = 3.0;

    public ShootCommand(ShooterSubsystem shooter, AgitatorSubsystem agitator) {
        shooterSubsystem = shooter;
        agitatorSubsystem = agitator;
        addRequirements(shooterSubsystem, agitatorSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        shooterSubsystem.setShooterWheelsShoot();
    }

    public void execute() {
        if (timer.get() < SHOOTTIME) { 
        agitatorSubsystem.agitateIfShootSpeed();
        }
        else {
            agitatorSubsystem.stopAgitating();
        }
    }

    public boolean isFinished() {
        if(timer.get() > SHOOTTIME) {
            return true;
        }
        else {
            return false;
        }
    }

}
