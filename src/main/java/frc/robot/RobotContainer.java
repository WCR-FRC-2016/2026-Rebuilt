package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.LimelightAlign;

import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer {

    final CommandXboxController driverCommandXbox = new CommandXboxController(0);
    final XboxController driverXbox = new XboxController(0);

    final CommandXboxController manipulatorCommandXbox = new CommandXboxController(1);
    final XboxController manipulatorXbox = new XboxController(1);

    private final XboxController controller = new XboxController(2);

    private final SwerveSubsystem drivebase = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    private final CollectorSubsystem collector = new CollectorSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final LedSubsystem ledSubsystem = new LedSubsystem();
    private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    SwerveInputStream driveDirectAngle = driveAngularVelocity
            .copy()
            .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
            .headingWhile(true);

    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    public RobotContainer() {

        configureBindings();

        DriverStation.silenceJoystickConnectionWarning(true);

        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("LimelightAlign", new LimelightAlign(drivebase));
    }

    @SuppressWarnings("unused")
    private void configureBindings() {

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        driverCommandXbox.y().whileTrue(new LimelightAlign(drivebase));

        driverCommandXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

        // Collector pivot
        manipulatorCommandXbox
                .x()
                .whileTrue(Commands.run(() -> collector.pivotCollectorUp(), collector))
                .onFalse(Commands.runOnce(() -> collector.stopPivotizing(), collector));

        manipulatorCommandXbox
                .y()
                .whileTrue(Commands.run(() -> collector.pivotCollectorDown(), collector))
                .onFalse(Commands.runOnce(() -> collector.stopPivotizing(), collector));

        // Intake
        manipulatorCommandXbox
                .leftTrigger(0.5)
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    collector.startCollecting();
                                    agitatorSubsystem.agitate(0.5);
                                },
                                collector,
                                agitatorSubsystem))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    collector.stopCollecting();
                                    agitatorSubsystem.stop();
                                },
                                collector,
                                agitatorSubsystem));

        // Reverse intake
        manipulatorCommandXbox
                .leftBumper()
                .whileTrue(
                        Commands.runOnce(
                                () -> {
                                    collector.startReleasing();
                                    agitatorSubsystem.agitate(0.5);
                                },
                                collector,
                                agitatorSubsystem))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    collector.stopCollecting();
                                    agitatorSubsystem.stop();
                                },
                                collector,
                                agitatorSubsystem));
        driverCommandXbox
                .x()
                .whileTrue(
                        Commands.run(
                                () -> {
                                    agitatorSubsystem.agitate(0.5);
                                },
                                shooter))
                .onFalse(
                        Commands.run(
                                () -> {
                                    agitatorSubsystem.stop();
                                },
                                shooter)
                                );
        driverCommandXbox
                .y()
                .whileTrue(
                        Commands.run(
                                () -> {
                                    agitatorSubsystem.agitate(-0.5);
                                },
                                shooter))
                .onFalse(
                        Commands.run(
                                () -> {
                                    agitatorSubsystem.stop();
                                },
                                shooter)
                                );
                                

        // Shooter wheels
        driverCommandXbox
                .rightTrigger(0.1)
                .whileTrue(
                        Commands.run(
                                () -> {
                                    double velocity = shooter.ShooterWheelsRun(-1);    
                                    if (velocity >= 41.6) {
                                        agitatorSubsystem.agitate(0.5);
                                    }
                                },
                                shooter))
                .onFalse(Commands.runOnce(() -> shooter.ShooterWheelsStop(), shooter));

        // Shooter pivot up
        driverCommandXbox
                .a()
                .whileTrue(Commands.run(() -> shooter.pivotUp(0.2), shooter))
                .onFalse(Commands.runOnce(() -> shooter.stopPivotizing(), shooter));

        // Shooter pivot down
        driverCommandXbox
                .b()
                .whileTrue(Commands.run(() -> shooter.pivotDown(-0.1), shooter))
                .onFalse(Commands.runOnce(() -> shooter.stopPivotizing(), shooter));

        // Climber
        manipulatorCommandXbox
                .povUp()
                .whileTrue(
                        Commands.startEnd(
                                () -> climberSubsystem.runClimber(),
                                () -> climberSubsystem.stop(),
                                climberSubsystem));

        manipulatorCommandXbox
                .povDown()
                .whileTrue(
                        Commands.startEnd(
                                () -> climberSubsystem.runClimber(),
                                () -> climberSubsystem.stop(),
                                climberSubsystem));

        // ---------------- LED controls ----------------
        new JoystickButton(controller, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(() -> ledSubsystem.setUp()));

        new JoystickButton(controller, XboxController.Button.kA.value)
                .onTrue(new InstantCommand(() -> ledSubsystem.setDown()));

        new JoystickButton(controller, XboxController.Button.kX.value)
                .onTrue(new InstantCommand(() -> ledSubsystem.setLeft()));

        new JoystickButton(controller, XboxController.Button.kB.value)
                .onTrue(new InstantCommand(() -> ledSubsystem.setRight()));

        new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
                .onTrue(new InstantCommand(() -> ledSubsystem.increaseSpeed()));

        new JoystickButton(controller, XboxController.Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> ledSubsystem.decreaseSpeed()));
    }

    public LedSubsystem getLedSubsystem() {
        return ledSubsystem;
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Left-Start");
    }

    public void setMotorBrake(boolean brake) {
        // drivebase.setMotorBrake(brake);
    }
}