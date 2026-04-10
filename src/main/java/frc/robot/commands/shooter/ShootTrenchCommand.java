package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootTrenchCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final AgitatorSubsystem agitatorSubsystem;
    Timer timer = new Timer();
    final double SHOOTTIME = 3.0;

    public ShootTrenchCommand(ShooterSubsystem shooter, AgitatorSubsystem agitator) {
        shooterSubsystem = shooter;
        agitatorSubsystem = agitator;
        addRequirements(shooterSubsystem, agitatorSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        shooterSubsystem.setShooterWheelsShootTrench();
    }

    public void execute() {
        if (timer.get() < SHOOTTIME) { 
        agitatorSubsystem.agitateIfShootSpeed();
        shooterSubsystem.feedFlyWheels();
        }
        else {
            agitatorSubsystem.stopAgitating();
            shooterSubsystem.feedFlyWheelsStop();
        }
    }

    public boolean isFinished() {
        if(timer.get() > SHOOTTIME) {
            shooterSubsystem.stopShooterWheels();
            return true;
        }
        else {
            return false;
        }
    }

}
