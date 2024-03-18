package frc.robot;

import frc.robot.Constants.OperatorConstant;
import frc.robot.commands.SpeakerShot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.PowerHubSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision.AprilTagDetectionSubsystem;
import frc.robot.subsystems.vision.GamePieceDetectionSubsystem;
import frc.robot.util.DataTracker;
import frc.robot.util.constants.LogConstants;
import frc.robot.util.constants.SwerveConstants.AutoConstants;
import frc.robot.util.constants.SwerveConstants.SwerveDrive;
import frc.robot.util.constants.VisionConstants.CameraTracking;
import frc.robot.util.swerve.SwerveUtils;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.trajectory.Trajectory;

/**
 * RobotContainer is the class where the bulk of the robot's systems are
 * declared.
 * Here, subsystems, OI devices, and commands are set up and should be the only
 * place that this configuration exists in the code.
 */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final CommandXboxController sysIdController;

  // private final AprilTagDetectionSubsystem aprilTagDetectionSubsystem;
  // private final GamePieceDetectionSubsystem gamePieceDetectionSubsystem;

  private SendableChooser<Command> autoChooser;

  private final PowerHubSubsystem powerHubSubsystem;

  private final DriveSubsystem driveSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  /**
   * Constructs the container for the robot. Subsystems and command mappings are
   * initialized here.
   */
  public RobotContainer() {

    // Start data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Port forwarding
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Initialize data tracker
    DataTracker.putBoolean(LogConstants.ROBOT_SYSTEM, "Intialization", true, false);

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OperatorConstant.OPERATOR_CONTROLLER_PORT);
    sysIdController = new CommandXboxController(OperatorConstant.SYSID_CONTROLLER_PORT);

    // Initialize helpers
    powerHubSubsystem = new PowerHubSubsystem();
    powerHubSubsystem.reset();

    // Initialize vision subsystems
    // aprilTagDetectionSubsystem = new AprilTagDetectionSubsystem(CameraTracking.APRIL_TAG_CAMERA_CONFIG);
    // gamePieceDetectionSubsystem = new GamePieceDetectionSubsystem(CameraTracking.GAME_PIECE_CAMERA_CONFIG, powerHubSubsystem);

    // Initialize subsystems
    driveSubsystem = new DriveSubsystem();

    conveyorSubsystem = new ConveyorSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();

    // Configuration
    configurePathPlanner();
    configureDriverBindings();
    configureOperatorBindings();
    configureSysIdBindings();
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureDriverBindings() {
    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveSubsystem.drive(
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstant.DEAD_BAND)),
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstant.DEAD_BAND)),
                -SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getRightX(), OperatorConstant.DEAD_BAND)),
                true, true),
            driveSubsystem));
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureOperatorBindings() {

    // Set elevator to drive position, really shouldn't be used unless the operator messed up and pressed the climb button
    operatorController.povDown().onTrue(new RunCommand(() -> elevatorSubsystem.drivePosition(), elevatorSubsystem)
        .until(() -> elevatorSubsystem.isElevatorAtPosition()));

    operatorController.povLeft().onTrue(new RunCommand(() -> elevatorSubsystem.decreasePosition(), elevatorSubsystem)
        .until(() -> elevatorSubsystem.isElevatorAtPosition()));

    operatorController.povRight().onTrue(new RunCommand(() -> elevatorSubsystem.increasePosition(), elevatorSubsystem)
        .until(() -> elevatorSubsystem.isElevatorAtPosition()));

    // Intake note
    operatorController.a().onTrue(new InstantCommand(intakeSubsystem::toggleIntake, intakeSubsystem));

    // AMP note discharge
    // operatorController.b().onTrue(
    //     new SequentialCommandGroup(
    //         // Run intake, conveyor, shooter in parallel until the game piece is ready
    //         new ParallelCommandGroup(
    //             new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
    //             new RunCommand(() -> conveyorSubsystem.runConveyorForwardAmp(), conveyorSubsystem),
    //             new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem))
    //             .until(() -> conveyorSubsystem.isGamePieceAmpReady()),

    //         // Stop intake, conveyor and shooter
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
    //             new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
    //             new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)),

    //         // Then, move the elevator to amp position
    //         new RunCommand(() -> elevatorSubsystem.ampPosition(), elevatorSubsystem)
    //             .until(() -> elevatorSubsystem.isElevatorAtPosition())));

                
    // operatorController.x().onTrue(
    //     new SequentialCommandGroup(
    //         // Run the conveyor and shooter again to discharge the game piece
    //         new ParallelCommandGroup(
    //             new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem),
    //             new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem))
    //             .until(() -> !conveyorSubsystem.isGamePieceAmpReady()),

    //         // Finally, stop the conveyor and shooter
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
    //             new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)),

    //         // Then, move the elevator to drive position
    //         new RunCommand(() -> elevatorSubsystem.drivePosition(), elevatorSubsystem)
    //             .until(() -> elevatorSubsystem.isElevatorAtPosition())));

    // Speaker note shooting
    operatorController.y().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> shooterSubsystem.resetState(), shooterSubsystem),
            // Get shooter rollers up to speed
            new RunCommand(() -> shooterSubsystem.flyWheelFullSpeed(), shooterSubsystem)
                .until(() -> shooterSubsystem.isShooterAtSpeed()),
            new WaitCommand(0.2),
            // Run intake, conveyor, shooter in parallel until the game piece is ready
            new ParallelCommandGroup(
                new RunCommand(() -> shooterSubsystem.flyWheelFullSpeed(), shooterSubsystem),
                new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
                new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem))
                .until(() -> shooterSubsystem.hasGamePieceBeenShot()),
            new WaitCommand(0.2),
            // Stop intake, conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
                new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
                new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem))));
  }

  private void configureSysIdBindings() {
    // Intake
    // sysIdController.a().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // sysIdController.b().whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // sysIdController.x().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // sysIdController.y().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void configurePathPlanner() {
    // Register commands
    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake(intakeSubsystem));
    NamedCommands.registerCommand("SpeakerShot", new SpeakerShot(intakeSubsystem, conveyorSubsystem, shooterSubsystem));

    // Build an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("SpeakerCentre_Quad");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Gets the command to run in autonomous mode.
   *
   * @return The autonomous command to run.`
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
