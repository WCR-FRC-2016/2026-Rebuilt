package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Rumble;
import frc.robot.commands.agitator.Agitate;
import frc.robot.commands.agitator.ShootAgitate;
import frc.robot.commands.climber.ClimbAuto;
import frc.robot.commands.collector.StartCollectingAuto;
import frc.robot.commands.collector.StopCollectingAuto;
import frc.robot.commands.collector.movePivotDown;
import frc.robot.commands.shooter.MovePivot;
import frc.robot.commands.shooter.calculateAndMovePivot;
import frc.robot.commands.swervedrive.drivebase.LimelightAlign;

import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import javax.sound.sampled.SourceDataLine;

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

  private final CollectorSubsystem collector = new CollectorSubsystem(
      () -> MathUtil.applyDeadband(manipulatorCommandXbox.getLeftY(), 0.4) * -1);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final LedSubsystem ledSubsystem = new LedSubsystem();
  private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final Trigger leftJoystickManualTrigger = new Trigger(
      () -> Math.abs(manipulatorCommandXbox.getLeftY()) > 0.4);

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
    NamedCommands.registerCommand("LimelightAlign", new LimelightAlign(drivebase, shooter));
    // NamedCommands.registerCommand("startCollecting", StartCollecting());

  }

  @SuppressWarnings("unused")
  private void configureBindings() {
    registerAutos();

    NamedCommands.registerCommand("Agitate", new Agitate(agitatorSubsystem, Constants.SpeedConstants.AGITATOR_SPEED));
    NamedCommands.registerCommand("StartCollectingAuto", new StartCollectingAuto(collector));
    NamedCommands.registerCommand("StopCollectingAuto", new StopCollectingAuto(collector));
    NamedCommands.registerCommand("Climb", new ClimbAuto(climberSubsystem));
    NamedCommands.registerCommand("ShootAlign", new LimelightAlign(drivebase, shooter));
    NamedCommands.registerCommand("movePivotDown", new movePivotDown(collector));

    DriverStation.silenceJoystickConnectionWarning(true);

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

  // public Command getAutonomousCommand() {
  // return AutoBuilder.buildAuto("Left-Start");
  // }

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }

  private void bindTestingControls() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    driverCommandXbox.x().onTrue(new calculateAndMovePivot(shooter));

    driverCommandXbox.y().whileTrue(new LimelightAlign(drivebase, shooter));

    driverCommandXbox.back().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Shooter wheels ramp up
    driverCommandXbox
        .leftTrigger(0.1)
        .onTrue(Commands.runOnce(shooter::ShooterWheelsRunSlow))
        .onFalse(Commands.runOnce(shooter::ShooterWheelsStop));
        
        //.whileTrue(Commands.run(shooter::ShooterWheelsRun, shooter))
        //.whileTrue(new Rumble(manipulatorXbox, 0.5))
        //.onFalse(Commands.runOnce(shooter::ShooterWheelsStop, shooter));

    // Agitate to shoot
    driverCommandXbox
        .rightTrigger(0.1)
        .whileTrue(Commands.run(() -> {
          if(shooter.isUpToSpeed()) {
          agitatorSubsystem.startAgitating();
          }
        }, agitatorSubsystem))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating, agitatorSubsystem));
    /*
     * driverCommandXbox
     * .rightTrigger(0.1)
     * .whileTrue(Commands.runOnce(new ShootAgitate(agitatorSubsystem,
     * Constants.AGITATOR_SPEED)))
     * .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating,
     * agitatorSubsystem));
     */

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
    /* */ driverCommandXbox
        .a()
        .onTrue(Commands.runOnce(() -> shooter.changeSpeedUp(), shooter));

    // Shooter speed down
    driverCommandXbox.b()
        .onTrue(Commands.runOnce(() -> shooter.changeSpeedDown(), shooter));
    // Reverse Agitator!!!
    driverCommandXbox.rightBumper()
        .whileTrue(Commands.runOnce(agitatorSubsystem::reverseAgitating, agitatorSubsystem))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating, agitatorSubsystem));

    // Collector pivot up
    /*
     * manipulatorCommandXbox
     * .leftBumper()
     * .whileTrue(Commands.run(collector::pivotCollectorUp, collector))
     * .onFalse(Commands.runOnce(collector::stopPivotizing, collector));
     * 
     * // Collector pivot down
     * manipulatorCommandXbox
     * .rightBumper()
     * .whileTrue(Commands.run(collector::pivotCollectorDown, collector))
     * .onFalse(Commands.runOnce(collector::stopPivotizing, collector));
     */
    manipulatorCommandXbox.a().onTrue(Commands.runOnce(collector::zeroPivotEncoder));
    manipulatorCommandXbox.leftBumper().onTrue(Commands.runOnce(collector::setPivotDown));
    manipulatorCommandXbox.rightBumper()
        .onTrue(Commands.runOnce(collector::setPivotUp))
        .whileTrue(Commands.runOnce(collector::startSpiting))
        .onFalse(Commands.runOnce(collector::stopCollection));
    leftJoystickManualTrigger.whileTrue(Commands.run(collector::setPivotManually));

    // Intake
    manipulatorCommandXbox
        .leftTrigger(0.1)
        // .onTrue(Commands.runOnce(collector:: setPivotDown))
        .whileTrue(Commands.runOnce(collector::startCollecting, collector))
        .onFalse(Commands.runOnce(collector::stopCollection, collector));

    // Reverse intake
    manipulatorCommandXbox
        .rightTrigger(0.1)
        .whileTrue(Commands.runOnce(collector::startSpiting, collector))
        .onFalse(Commands.runOnce(collector::stopCollection, collector));

    manipulatorCommandXbox.y().onTrue(Commands.runOnce(collector::setPivotShoot, collector));

    // Climber
    manipulatorCommandXbox
        .povUp()
        .whileTrue(Commands.run(climberSubsystem::runClimber, climberSubsystem))
        .onFalse(Commands.run(climberSubsystem::stop, climberSubsystem));

    manipulatorCommandXbox
        .povDown()
        .whileTrue(Commands.run(climberSubsystem::runClimberDown, climberSubsystem))
        .onFalse(Commands.run(climberSubsystem::stop, climberSubsystem));
    manipulatorCommandXbox
        .b()
        .whileTrue(new MovePivot(shooter, -1));

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

  public Command getAutonomousCommand() {
    // return drivebase.driveCommand(() -> -0.1, () -> 0, () -> 0);

    var auto = SmartDashboard.getString("Auto Selector", "Test");
    System.out.println("Selected Autonomous: " + ((auto == null) ? "[null, cannot find one]" : auto));

    // Verify and autonomous command was able to be found from the Dashboard
    if (auto == null)
      return drivebase.getAutonomousCommand(Auton.DEFAULT_AUTO_NAME);

    // Verify the command actually exists. This will return the command if its in
    // the list. (MUST match)
    for (var i = 0; i < Auton.AUTO_NAMES.length; i++)
      if (Auton.AUTO_NAMES[i].equals(auto))
        return drivebase.getAutonomousCommand(auto);

    // Dont try to run a autonomous that isn't verifiably in the list
    System.out.println("This autonomous is not in the list!!!");
    System.out.println(" . Make sure you select one from the dropdown and DONT CHANGE IT.");
    System.out.println("    > It MUST be in the list to be run.");
    return drivebase.getAutonomousCommand(Auton.DEFAULT_AUTO_NAME);
  }

  private void registerAutos() {
    SmartDashboard.putStringArray("Auto List", Auton.AUTO_NAMES);
  }

  private void bindCompetitionControls() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    driverCommandXbox.rightTrigger().whileTrue(new LimelightAlign(drivebase, shooter));

    driverCommandXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

    // Shooter wheels ramp up
    driverCommandXbox
        .leftTrigger(0.1)
        .whileTrue(Commands.run(shooter::ShooterWheelsRun, shooter))
        .onFalse(Commands.runOnce(shooter::ShooterWheelsStop, shooter));

    // Agitate to shoot
    driverCommandXbox
        .leftBumper()
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

    // Reverse Agitator!!!
    driverCommandXbox.b()
        .whileTrue(Commands.runOnce(agitatorSubsystem::reverseAgitating, agitatorSubsystem))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating, agitatorSubsystem));

    // Collector pivot up
    /*
     * manipulatorCommandXbox
     * .leftBumper()
     * .whileTrue(Commands.run(collector::pivotCollectorUp, collector))
     * .onFalse(Commands.runOnce(collector::stopPivotizing, collector));
     * 
     * // Collector pivot down
     * manipulatorCommandXbox
     * .rightBumper()
     * .whileTrue(Commands.run(collector::pivotCollectorDown, collector))
     * .onFalse(Commands.runOnce(collector::stopPivotizing, collector));
     */
    // manipulatorCommandXbox.a().onTrue(Commands.runOnce(collector::
    // zeroPivotEncoder));
    manipulatorCommandXbox.leftBumper().onTrue(Commands.runOnce(collector::setPivotDown));
    manipulatorCommandXbox.rightBumper()
        .onTrue(Commands.runOnce(collector::setPivotUp));
    manipulatorCommandXbox.y().whileTrue(Commands.run(agitatorSubsystem::reverseAgitating, agitatorSubsystem))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating, agitatorSubsystem));

    leftJoystickManualTrigger
        .whileTrue(Commands.run(collector::setPivotManually)); // change to pid if not already set

    // Intake
    manipulatorCommandXbox
        .leftTrigger(0.1)
        // .onTrue(Commands.runOnce(collector:: setPivotDown))
        .whileTrue(Commands.runOnce(collector::startCollecting, collector))
        .onFalse(Commands.runOnce(collector::stopCollection, collector));

    // Reverse intake
    manipulatorCommandXbox
        .rightTrigger(0.1)
        .whileTrue(Commands.runOnce(collector::startSpiting, collector))
        .onFalse(Commands.runOnce(collector::stopCollection, collector));

    // manipulatorCommandXbox.y().onTrue(Commands.runOnce(collector::setPivotShoot,
    // collector));

    // Climber
    manipulatorCommandXbox
        .povUp()
        .whileTrue(Commands.run(climberSubsystem::runClimber, climberSubsystem))
        .onFalse(Commands.run(climberSubsystem::stop, climberSubsystem));

    manipulatorCommandXbox
        .povDown()
        .whileTrue(Commands.run(climberSubsystem::runClimberDown, climberSubsystem))
        .onFalse(Commands.run(climberSubsystem::stop, climberSubsystem));

    manipulatorCommandXbox
    .x()
    .onTrue(Commands.run(shooter::ShooterWheelsRunBack, shooter))
    .onFalse(Commands.runOnce(shooter::ShooterWheelsStop, shooter));

  }

}