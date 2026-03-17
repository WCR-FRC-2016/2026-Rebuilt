package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Rumble;
import frc.robot.commands.collector.StartCollectingAuto;
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
        // for possible leds dont touch unless neccisary

        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve/neo"));

        private final CollectorSubsystem collector = new CollectorSubsystem(() -> MathUtil.applyDeadband(manipulatorCommandXbox.getLeftY(), 0.4)*-1);
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final LedSubsystem ledSubsystem = new LedSubsystem();
        private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();
        private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

        private final Trigger leftJoystickManualTrigger = new Trigger(() -> Math.abs(manipulatorCommandXbox.getLeftY()) > 0.4);

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

        boolean ifDone = false;

        public RobotContainer() {

                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);

                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                //NamedCommands.registerCommand("LimelightAlign", new LimelightAlign(drivebase));
                // NamedCommands.registerCommand("startCollecting", StartCollecting());

        }

        @SuppressWarnings("unused")
        private void configureBindings() {

                // if (DriverStation.isTest()) {
                bindTestingControls();
                // } else {
                // bindCompetitionControls();
                // }

                /*
                 * // ---------------- LED controls ----------------
                 * new JoystickButton(controller, XboxController.Button.kY.value)
                 * .onTrue(new InstantCommand(() -> ledSubsystem.setUp()));
                 * 
                 * new JoystickButton(controller, XboxController.Button.kA.value)
                 * .onTrue(new InstantCommand(() -> ledSubsystem.setDown()));
                 * 
                 * new JoystickButton(controller, XboxController.Button.kX.value)
                 * .onTrue(new InstantCommand(() -> ledSubsystem.setLeft()));
                 * 
                 * new JoystickButton(controller, XboxController.Button.kB.value)
                 * .onTrue(new InstantCommand(() -> ledSubsystem.setRight()));
                 * 
                 * new JoystickButton(controller, XboxController.Button.kLeftBumper.value)
                 * .onTrue(new InstantCommand(() -> ledSubsystem.increaseSpeed()));
                 * 
                 * new JoystickButton(controller, XboxController.Button.kRightBumper.value)
                 * .onTrue(new InstantCommand(() -> ledSubsystem.decreaseSpeed()));
                 */
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

        private void bindTestingControls() {

                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

                Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

                drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

                driverCommandXbox.y().whileTrue(new LimelightAlign(drivebase));

                driverCommandXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

                // Shooter wheels ramp up
                driverCommandXbox
                                .leftTrigger(0.1)
                                .whileTrue(Commands.run(shooter::ShooterWheelsRun, shooter))
                                .whileTrue(new Rumble(manipulatorXbox, 0.5))
                                .onFalse(Commands.runOnce(shooter::ShooterWheelsStop, shooter));

                // Agitate to shoot
                driverCommandXbox
                                .rightTrigger(0.1)
                                .whileTrue(Commands.run(agitatorSubsystem::startAgitating, agitatorSubsystem))
                                .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating, agitatorSubsystem));

                // Shooter pivot up
                driverCommandXbox
                                .povUp()
                                .whileTrue(Commands.run(shooter::pivotUp, shooter))
                                .onFalse(Commands.runOnce(shooter::stopPivotizing, shooter));

                // Shooter pivot down
                driverCommandXbox
                                .povDown()
                                .whileTrue(Commands.run(shooter::pivotDown, shooter))
                                .onFalse(Commands.runOnce(shooter::stopPivotizing, shooter));
                // Shooter speed up
              /* */  driverCommandXbox
                                .a()
                                .onTrue(Commands.runOnce(() -> shooter.changeSpeedUp(), shooter));

                // Shooter speed down
                driverCommandXbox.b()
                .onTrue(Commands.runOnce(() -> shooter.changeSpeedDown(), shooter));

                // Collector pivot up
                /* manipulatorCommandXbox
                                .leftBumper()
                                .whileTrue(Commands.run(collector::pivotCollectorUp, collector))
                                .onFalse(Commands.runOnce(collector::stopPivotizing, collector));

                // Collector pivot down
                manipulatorCommandXbox
                                .rightBumper()
                                .whileTrue(Commands.run(collector::pivotCollectorDown, collector))
                                .onFalse(Commands.runOnce(collector::stopPivotizing, collector)); */
                manipulatorCommandXbox.a().onTrue(Commands.runOnce(collector:: zeroPivotEncoder));
                manipulatorCommandXbox.leftBumper().onTrue(Commands.runOnce(collector::setPivotDown));
                manipulatorCommandXbox.rightBumper()
                .onTrue(Commands.runOnce(collector::setPivotUp))
                .whileTrue(Commands.runOnce(collector::startSpiting))
                .onFalse(Commands.runOnce(collector:: stopCollection));
                leftJoystickManualTrigger. whileTrue(Commands.run(collector::setPivotManually));

                // Intake
                manipulatorCommandXbox
                                .leftTrigger(0.1)
                                .onTrue(Commands.runOnce(collector:: setPivotDown))
                                .whileTrue(Commands.runOnce(collector::startCollecting, collector))
                                .onFalse(Commands.runOnce(collector::stopCollection, collector));

                // Reverse intake
                manipulatorCommandXbox
                                .rightTrigger(0.1)
                                .whileTrue(Commands.runOnce(collector::startSpiting, collector))
                                .onFalse(Commands.runOnce(collector::stopCollection, collector));

                // Climber
                manipulatorCommandXbox
                                .povUp()
                                .whileTrue(Commands.run(climberSubsystem::runClimber, climberSubsystem))
                                .onFalse(Commands.run(climberSubsystem::stop, climberSubsystem));

                manipulatorCommandXbox
                                .povDown()
                                .whileTrue(Commands.run(climberSubsystem::runClimberDown, climberSubsystem))
                                .onFalse(Commands.run(climberSubsystem::stop, climberSubsystem));

                leftJoystickManualTrigger
                                .onTrue(Commands.runOnce(() -> System.out.println("Works!!")));
                // () -> collector.manualCollectorPivot()));
                /*
                 * if (manipulatorCommandXbox.getLeftY() > 0.4) {
                 * collector.manualCollectorPivot(manipulatorCommandXbox.getLeftY());
                 * }
                 * else {
                 * collector.stopPivotizing();
                 * }
                 */

        }

        private void bindCompetitionControls() {
                System.out.println("To do: bind competition controls.");
        }

}