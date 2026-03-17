package frc.robot.commands.swervedrive.agitator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ShootAgitate extends Command {
     private AgitatorSubsystem agitatorSubsystem;
     private ShooterSubsystem shooterSubsystem;
    double speed;


    public ShootAgitate(AgitatorSubsystem agitatorSubsystem, double speed) {
        this.agitatorSubsystem = agitatorSubsystem;

        this.speed = speed;
        addRequirements(agitatorSubsystem);
}
 @Override
public void execute() {
    if( Math.abs(shooterSubsystem.currentVelocity) >= Math.abs(shooterSubsystem.wantedVelocity) + 5 && Math.abs(shooterSubsystem.currentVelocity) <= Math.abs(shooterSubsystem.wantedVelocity) - 5){

    agitatorSubsystem.startAgitating();}
    else {
        agitatorSubsystem.stopAgitating();
    }

}
@Override
public void end(boolean interupted){
agitatorSubsystem.stopAgitating();//when set up a command add the uptospeed requirements




}
 
}