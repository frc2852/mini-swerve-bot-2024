package frc.robot;

import frc.robot.commands.Discharge;
import frc.robot.commands.SpeakerShot;
import frc.robot.commands.ToggleIntake;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.Constants.OperatorConstant;
import frc.robot.constants.Constants.SubsystemEnable;
import frc.robot.subsystems.PowerHub;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.GamePieceDetection;
import frc.robot.util.swerve.SwerveUtils;
import frc.robot.util.vision.Color;

import java.util.function.Function;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer is the class where the bulk of the robot's systems are
 * declared.
 * Here, subsystems, OI devices, and commands are set up and should be the only
 * place that this configuration exists in the code.
 */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private SendableChooser<Command> autoChooser;

  private final PowerHub powerHub;
  private final Drive drive;
  private final Conveyor conveyor;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final LEDs leds;

  // Vision
  private final GamePieceDetection gamePieceDetection;

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

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OperatorConstant.OPERATOR_CONTROLLER_PORT);

    // Initialize helpers
    powerHub = new PowerHub();

    // Initialize vision
    gamePieceDetection = new GamePieceDetection(VisionConstants.CameraTracking.GAME_PIECE_CAMERA_CONFIG);

    // Initialize subsystems
    drive = new Drive(gamePieceDetection);
    leds = new LEDs();
    leds.setLEDColor(Color.PINK);

    shooter = initSubsystem(SubsystemEnable.SHOOTER, Shooter::new, leds);
    conveyor = initSubsystem(SubsystemEnable.CONVEYOR, Conveyor::new);
    elevator = initSubsystem(SubsystemEnable.ELEVATOR, Elevator::new);
    intake = initSubsystem(SubsystemEnable.INTAKE, Intake::new, leds, powerHub);

    // Configuration
    configureBindings();
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureBindings() {
    if (drive != null) {
      configureDriverBindings();
    }

    if (conveyor != null && elevator != null && intake != null && shooter != null) {
      configureOperatorBindings();
      configurePathPlanner();
    }
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureDriverBindings() {
    // Climb
    driverController.povRight().onTrue(new RunCommand(() -> elevator.climbUpPosition(), elevator));
    driverController.povLeft().onTrue(new RunCommand(() -> elevator.climbDownPosition(), elevator));
    driverController.povUp().onTrue(new InstantCommand(() -> elevator.manualElevatorUp(), elevator));
    driverController.povDown().onTrue(new InstantCommand(() -> elevator.manualElevatorDown(), elevator));

    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drive.drive(
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstant.DEAD_BAND)),
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstant.DEAD_BAND)),
                -SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getRightX(), OperatorConstant.DEAD_BAND)),
                true, true),
            drive));
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureOperatorBindings() {

    // Stop shooter
    operatorController.rightBumper().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> shooter.stopShooter(), shooter),
      new InstantCommand(() -> conveyor.stopConveyor(), conveyor)));

    // Intake note
    operatorController.a().onTrue(new InstantCommand(intake::toggleIntake, intake));

    // AMP note prepare
    operatorController.b().onTrue(
        new SequentialCommandGroup(
            // Run intake, conveyor, shooter in parallel until the game piece is ready
            new ParallelCommandGroup(
                new RunCommand(() -> intake.runIntake(true), intake),
                new RunCommand(() -> conveyor.runConveyorForwardAmp(), conveyor),
                new RunCommand(() -> shooter.divertGamePiece(), shooter))
                .until(() -> conveyor.isGamePieceAmpReady()),

            // Stop intake, conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> intake.stopIntake(), intake),
                new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
                new InstantCommand(() -> shooter.stopShooter(), shooter)),

            // Then, move the elevator to amp position
            new RunCommand(() -> elevator.ampPosition(), elevator)
                .until(() -> elevator.isElevatorAtPosition())));

    // AMP note discharge
    operatorController.x().onTrue(
        new SequentialCommandGroup(
            // Run the conveyor and shooter again to discharge the game piece
            new ParallelCommandGroup(
                new RunCommand(() -> conveyor.runConveyorForward(), conveyor),
                new RunCommand(() -> shooter.divertGamePiece(), shooter))
                .until(() -> !conveyor.isGamePieceAmpReady()),

            // Wait for 0.5 seconds
            new WaitCommand(0.5),

            // Finally, stop the conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
                new InstantCommand(() -> shooter.stopShooter(), shooter)),

            // Then, move the elevator to drive position
            new RunCommand(() -> elevator.drivePosition(), elevator)
                .until(() -> elevator.isElevatorAtPosition())));

    // Speaker note shooting
    operatorController.y().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.resetState(), shooter),
            // Get shooter rollers up to speed
            new RunCommand(() -> shooter.flyWheelFullSpeed(), shooter)
                .until(() -> shooter.isShooterAtSpeed()),
            new WaitCommand(0.2),
            // Run intake, conveyor, shooter in parallel until the game piece is ready
            new ParallelCommandGroup(
                new RunCommand(() -> shooter.flyWheelFullSpeed(), shooter),
                new RunCommand(() -> intake.runIntake(true), intake),
                new RunCommand(() -> conveyor.runConveyorForward(), conveyor))
                .until(() -> shooter.hasGamePieceBeenShot()),
            new WaitCommand(0.2),
            // Stop intake, conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> intake.stopIntake(), intake),
                new InstantCommand(() -> conveyor.stopConveyor(), conveyor),
                new InstantCommand(() -> shooter.stopShooter(), shooter))));
  }

  private void configurePathPlanner() {
    // Register commands
    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake(intake));
    NamedCommands.registerCommand("SpeakerShot", new SpeakerShot(intake, conveyor, shooter));
    NamedCommands.registerCommand("Discharge", new Discharge(intake, conveyor, shooter));

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

  /**
   * Initializes a subsystem if it is enabled.
   *
   * @param <T>         The type of the subsystem that extends {@link SubsystemBase}.
   * @param enabled     A boolean indicating whether the subsystem should be initialized.
   * @param constructor A {@link Supplier} that provides the constructor for the subsystem.
   * @return An instance of the subsystem if enabled, otherwise {@code null}.
   */
  private <T extends SubsystemBase> T initSubsystem(boolean enabled, Supplier<T> constructor) {
    return enabled ? constructor.get() : null;
  }

  /**
   * Initializes a subsystem if it is enabled.
   *
   * @param <T>         The type of the subsystem that extends {@link SubsystemBase}.
   * @param enabled     A boolean indicating whether the subsystem should be initialized.
   * @param constructor A {@link Function} that takes an array of {@link Object} and returns the subsystem.
   * @param args        The arguments to pass to the subsystem constructor.
   * @return An instance of the subsystem if enabled, otherwise {@code null}.
   */
  private <T extends SubsystemBase> T initSubsystem(boolean enabled, Function<Object[], T> constructor, Object... args) {
    return enabled ? constructor.apply(args) : null;
  }

}
