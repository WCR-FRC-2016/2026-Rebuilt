package frc.robot.commands.swervedrive.agitator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Agitate extends Command {
     private AgitatorSubsystem agitatorSubsystem;
     private ShooterSubsystem shooterSubsystem;
    double speed;


    public Agitate(AgitatorSubsystem agitatorSubsystem, double speed) {
        this.agitatorSubsystem = agitatorSubsystem;

        this.speed = speed;
        addRequirements(agitatorSubsystem);
}
 @Override
public void execute() {
  //  agitatorSubsystem.agitate(speed);
    

}
@Override
public void end(boolean interupted){
//agitatorSubsystem.stop();//when set up a command add the uptospeed requirements




}
 
}