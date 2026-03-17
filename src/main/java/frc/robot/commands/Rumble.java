package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Rumble extends Command {
    XboxController xboxController;
    ShooterSubsystem shooterSubsystem;
    Double intensity;

    public Rumble(XboxController xboxController, Double intensity){
        this.xboxController = xboxController;
        this.intensity = intensity;
    }

    @Override
    public void execute(){
         while(shooterSubsystem.isUpToSpeed()== true){
        xboxController.setRumble(RumbleType.kBothRumble, intensity);
    }
}

    @Override
    public void end(boolean interrupted){
        xboxController.setRumble(RumbleType.kBothRumble, 0);
    }
}