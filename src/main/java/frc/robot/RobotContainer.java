package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Auton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.shooter.PassBallsCommand;
import frc.robot.commands.swervedrive.drivebase.LimelightAlignCommand;
import frc.robot.commands.swervedrive.drivebase.LimelightHoodAlignCommand;
import frc.robot.commands.swervedrive.drivebase.LimelightHoodAlignAutoCommand;
import frc.robot.subsystems.agitator.AgitatorSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.collector.CollectorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer {
  private final CommandXboxController driverCommandXbox = new CommandXboxController(0);
  private final CommandXboxController manipulatorCommandXbox = new CommandXboxController(1);

  private final SwerveSubsystem drivebaseSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final AgitatorSubsystem agitatorSubsystem = new AgitatorSubsystem(shooterSubsystem);
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(
    () -> MathUtil.applyDeadband(manipulatorCommandXbox.getLeftY(), 0.4) * -1
  );

  private final Trigger leftJoystickManualTrigger = new Trigger(
    () -> Math.abs(manipulatorCommandXbox.getLeftY()) > 0.4
  );

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebaseSubsystem.getSwerveDrive(),
      () -> driverCommandXbox.getLeftY() * -1,
      () -> driverCommandXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverCommandXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity
      .copy()
      .withControllerHeadingAxis(driverCommandXbox::getRightX, driverCommandXbox::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    DriverStation.silenceJoystickConnectionWarning(true);
  
    registerAutos();
    bindCompetitionControls();
  }

  // TODO: verify this works
  public Command getAutonomousCommand() {
    var auto = SmartDashboard.getString("Auto Selector", "Do Nothing");
    System.out.println("Selected Autonomous: " + ((auto == null) ? "[null, cannot find one]" : auto));

    // Verify and autonomous command was able to be found from the Dashboard
    if (auto == null)
      return drivebaseSubsystem.getAutonomousCommand(Auton.DEFAULT_AUTO_NAME);

    // Verify the command actually exists. This will return the command if its in
    // the list. (MUST match)
    for (var i = 0; i < Auton.AUTO_NAMES.length; i++) {
      System.out.println("Auto");
      return drivebaseSubsystem.getAutonomousCommand(auto);
    }

    // Dont try to run a autonomous that isn't verifiably in the list
    System.out.println("This autonomous is not in the list!!!");
    System.out.println(" . Make sure you select one from the dropdown and DONT CHANGE IT.");
    System.out.println("    > It MUST be in the list to be run.");
    return drivebaseSubsystem.getAutonomousCommand(Auton.DEFAULT_AUTO_NAME);
  }

  public void setMotorBrake(boolean isBraked){
    drivebaseSubsystem.setMotorBrake(isBraked);
  }

  private void registerAutos() {
    // TODO: Review command names
    SmartDashboard.putStringArray("Auto List", Auton.AUTO_NAMES);
    NamedCommands.registerCommand("StartShooterWheels", Commands.runOnce(shooterSubsystem::setShooterWheelsShoot));
    NamedCommands.registerCommand("StopShooterWheels", Commands.runOnce(shooterSubsystem::stopShooterWheels));
    NamedCommands.registerCommand("LimelightAlign", new LimelightAlignCommand(drivebaseSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("AgitateIfAtSpeedUntilCancelled", Commands.run(agitatorSubsystem::agitateIfShootSpeed));
    NamedCommands.registerCommand("StopAgitate", Commands.run(agitatorSubsystem::stopAgitating));
    NamedCommands.registerCommand("PivotDown", Commands.runOnce(collectorSubsystem::setPivotDown));
    NamedCommands.registerCommand("PivotUp", Commands.runOnce(collectorSubsystem::setPivotUp));
    NamedCommands.registerCommand("RunClimberDown", Commands.runOnce(climberSubsystem::runClimberDown));
    NamedCommands.registerCommand("RunClimberUp", Commands.runOnce(climberSubsystem::runClimberUp));
    NamedCommands.registerCommand("LimelightHoodAlign", new LimelightHoodAlignCommand(drivebaseSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("Agitate", Commands.runOnce(agitatorSubsystem::startAgitating));
  }

  private void bindCompetitionControls() {
    // Right Trigger: Spin up shooter
    // Right Bumper: Feed into shooter (run agitator forward)
    // Y: Limelight Align
    // X: Auto Pivot (shoot anywhere)
    // Left Bumper: Feed out of shooter (run agitator backward)
    // POV UP: Shooter Hood Up (manual)
    // POV DOWN: Shooter Hood Down (manual)
    // Start: Zero Gyro
    Command driveFieldOrientedAngularVelocity = drivebaseSubsystem.driveFieldOriented(driveAngularVelocity);
    drivebaseSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

    driverCommandXbox.leftTrigger(0.2)
        .onTrue(Commands.runOnce(shooterSubsystem::setShooterWheelsReverse))
        .onFalse(Commands.runOnce(shooterSubsystem::stopShooterWheels));
    driverCommandXbox.rightTrigger(0.2)
        .onTrue(Commands.runOnce(shooterSubsystem::setShooterWheelsShoot))
        .onFalse(Commands.runOnce(shooterSubsystem::stopShooterWheels));
    driverCommandXbox.leftBumper()
        .whileTrue(new PassBallsCommand(shooterSubsystem, agitatorSubsystem));
    driverCommandXbox.rightBumper()
        .whileTrue(Commands.run(agitatorSubsystem::agitateIfShootSpeed))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating));
    driverCommandXbox.povLeft()
        .onTrue(Commands.runOnce(agitatorSubsystem::reverseAgitating))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating));
    driverCommandXbox.povRight()
        .whileTrue(Commands.run(agitatorSubsystem::startAgitating))
        .onFalse(Commands.runOnce(agitatorSubsystem::stopAgitating));
    driverCommandXbox.povUp()
        .onTrue(Commands.runOnce(shooterSubsystem::pivotUpManually))
        .onFalse(Commands.runOnce(shooterSubsystem::stopPivotingManually));
    driverCommandXbox.povDown()
        .onTrue(Commands.runOnce(shooterSubsystem::pivotDownManually))
        .onFalse(Commands.runOnce(shooterSubsystem::stopPivotingManually));
    driverCommandXbox.y()
        .whileTrue(new LimelightHoodAlignAutoCommand(drivebaseSubsystem, shooterSubsystem));
    driverCommandXbox.x()
        .whileTrue(new LimelightAlignCommand(drivebaseSubsystem, shooterSubsystem));
    driverCommandXbox.start()
        .onTrue(Commands.runOnce(drivebaseSubsystem::zeroGyro));

    manipulatorCommandXbox.x()
        .whileTrue(Commands.run(shooterSubsystem::setShooterWheelsReverse))
        .onFalse(Commands.runOnce(shooterSubsystem::stopShooterWheels));
    manipulatorCommandXbox.y()
        .whileTrue(Commands.run(agitatorSubsystem::reverseAgitating, agitatorSubsystem))
        .onFalse(Commands.run(agitatorSubsystem::stopAgitating, agitatorSubsystem));
    manipulatorCommandXbox.leftBumper()
        .onTrue(Commands.runOnce(collectorSubsystem::setPivotDown));
    manipulatorCommandXbox.rightBumper()
        .onTrue(Commands.runOnce(collectorSubsystem::setPivotUp));
    manipulatorCommandXbox.leftTrigger()
        .onTrue(Commands.runOnce(collectorSubsystem::startCollecting))
        .onFalse(Commands.runOnce(collectorSubsystem::stopCollection));
    manipulatorCommandXbox.rightTrigger()
        .onTrue(Commands.runOnce(collectorSubsystem::startSpitting))
        .onFalse(Commands.runOnce(collectorSubsystem::stopCollection));
    manipulatorCommandXbox.povUp()
        .onTrue(Commands.runOnce(climberSubsystem::runClimberUp))
        .onFalse(Commands.runOnce(climberSubsystem::climberStop));
    manipulatorCommandXbox.povDown()
        .onTrue(Commands.runOnce(climberSubsystem::runClimberDown))
        .onFalse(Commands.runOnce(climberSubsystem::climberStop));
    leftJoystickManualTrigger
        .whileTrue(Commands.run(collectorSubsystem::setPivotManually));

  }




}