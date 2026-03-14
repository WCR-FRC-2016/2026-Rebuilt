package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class PlayMusicCommand extends Command { // Renamed to avoid conflict

    private final Orchestra m_orchestra = new Orchestra();

    public PlayMusicCommand(TalonFX shooterLeader) { // Pass motor in constructor
        // Add a single device to the orchestra
        m_orchestra.addInstrument(shooterLeader);

        // Attempt to load the chrp
        StatusCode status = m_orchestra.loadMusic("botw.chrp"); // Declare type

        if (!status.isOK()) {
            System.err.println("Failed to load music: " + status.toString());
        }
    }

    @Override
    public void initialize() {
        m_orchestra.play(); // Actually start playing
    }

    @Override
    public void end(boolean interrupted) {
        m_orchestra.stop();
    }

    @Override
    public boolean isFinished() {
        return !m_orchestra.isPlaying();
    }
}