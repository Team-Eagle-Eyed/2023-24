package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final Joystick buttonBoard = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int speedAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverIntake = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton resetWheels = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Operator Controls */
    private final int armAxis = XboxController.Axis.kLeftY.value;
    private final int intakeAxis = XboxController.Axis.kLeftTrigger.value;
    private final int outtakeAxis = XboxController.Axis.kRightTrigger.value;

    /* Operator Buttons */
    private final JoystickButton launchNote = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton reverseIntake = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Button Board Buttons */
    private final JoystickButton estop = new JoystickButton(buttonBoard, 1);
    private final JoystickButton raiseArm = new JoystickButton(buttonBoard, 2);
    private final JoystickButton resetArm = new JoystickButton(buttonBoard, 3);
    private final JoystickButton launchNoteButtonBoard = new JoystickButton(buttonBoard, 4);
    private final JoystickButton goToNote = new JoystickButton(buttonBoard, 5);
    private final JoystickButton ampShot = new JoystickButton(buttonBoard, 6);
    private final JoystickButton relayShot = new JoystickButton(buttonBoard, 7);
    private final JoystickButton playMusic = new JoystickButton(buttonBoard, 10);

    /* Subsystems */
    private final ApriltagCamera s_ApriltagCamera = new ApriltagCamera(
                                                    "Apriltag Camera",
                                                    25,
                                                    1.45,
                                                    21.75
                                                    );
    private final Swerve s_Swerve = new Swerve(s_ApriltagCamera);
    private final NoteCamera s_NoteCamera = new NoteCamera("Note Camera");
    private final Arm s_Arm = new Arm(s_ApriltagCamera);
    private final Intake s_Intake = new Intake();
    private final Outtake s_Outtake = new Outtake();

    /* Sendable Choosers */
    private final SendableChooser<String> musicSelector = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        NamedCommands.registerCommand("launchNote", new LaunchNote(s_Outtake, s_Intake, () -> SmartDashboard.getNumber("Launcher set velocity", 4500)));
        NamedCommands.registerCommand("startIntake", new TeleopIntake(s_Intake, () -> 4000, true));
        NamedCommands.registerCommand("stopIntake", new TeleopIntake(s_Intake, () -> 0, true));
        NamedCommands.registerCommand("lowerArm", new SetArmPosition(s_Arm, () -> 23));
        NamedCommands.registerCommand("goToNote", new GoToNote(s_Swerve, s_Intake, s_NoteCamera));
        NamedCommands.registerCommand("alignForAuto", new TurnToAngle(
            s_Swerve,
            () -> {
                if(DriverStation.getAlliance().isPresent()) {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 90 : -90;
                } else {
                    return 90;
                }}
            ));
        NamedCommands.registerCommand("alignForShot", new TurnToAngle(s_Swerve, () -> 0));

        musicSelector.setDefaultOption("Imperial March", "imperial_march.chrp");
        musicSelector.addOption("Megalovania", "megalovania.chrp");
        musicSelector.addOption("Night on Bald Mountain", "night_on_bald_mountain.chrp");
        musicSelector.addOption("Sandstorm", "sandstorm.chrp");
        musicSelector.addOption("Mii Channel", "mii_channel.chrp");
        musicSelector.addOption("William Tell Overture", "william_tell_overture_finale.chrp");

        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putNumber("SpeedLimit", 1);
        SmartDashboard.putNumber("ShooterSpeed", 1);
        SmartDashboard.putData("Music Selector", musicSelector);
        SmartDashboard.putData("Auto Chooser", new SendableChooser<>()); // Clear previous options?
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putNumber("Launcher set velocity", 4500);
        SmartDashboard.putNumber("targetHeight", 95); // 84

        DriverStation.silenceJoystickConnectionWarning(true);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                s_Intake,
                s_ApriltagCamera,
                () -> -driver.getRawAxis(translationAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
                () -> -driver.getRawAxis(strafeAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
                () -> -driver.getRawAxis(rotationAxis) * 0.60 * SmartDashboard.getNumber("SpeedLimit", 1),
                () -> robotCentric.getAsBoolean(),
                () -> false
            )
        );

        s_Arm.setDefaultCommand(
            new TeleopArm(
                s_Arm,
                s_Intake,
                () -> -operator.getRawAxis(armAxis)
            )
        );

        s_Intake.setDefaultCommand(
            new TeleopIntake(
                s_Intake,
                () -> operator.getRawAxis(intakeAxis) * 4000,
                true
            )
        );

        s_Outtake.setDefaultCommand(
            new TeleopOuttake(
                s_Outtake,
                () -> operator.getRawAxis(outtakeAxis) * SmartDashboard.getNumber("Launcher set velocity", 1),
                true
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        s_ApriltagCamera.addPositionListener(s_Swerve);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driverIntake.whileTrue(new TeleopIntake(s_Intake, () -> SmartDashboard.getNumber("Launcher set velocity", 4000), true));
        launchNote.whileTrue(new LaunchNote(
                                    s_Outtake,
                                    s_Intake,
                                    () -> SmartDashboard.getNumber("Launcher set velocity", 4050)
                                    )
                                );
        launchNoteButtonBoard.whileTrue(new LaunchNote(
                                    s_Outtake,
                                    s_Intake,
                                    () -> SmartDashboard.getNumber("Launcher set velocity", 4050)
                                    )
                                );
        reverseIntake.whileTrue(new TeleopIntake(s_Intake, () -> -0.5, false));
        reverseIntake.whileTrue(new TeleopOuttake(s_Outtake, () -> -0.25, false));
        resetWheels.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        raiseArm.whileTrue(new SetArmPosition(s_Arm, () -> 57));
        ampShot.whileTrue(new AmpShot(s_Arm, s_Intake, s_Outtake));
        relayShot.whileTrue(new RelayShot(s_Swerve, s_Arm, s_Intake, s_Outtake));
        resetArm.whileTrue(new TeleopArm(s_Arm, s_Intake, () -> -0.2));
        goToNote.whileTrue(new GoToNote(s_Swerve, s_Intake, s_NoteCamera));
        playMusic.whileTrue(new MusicPlayer(s_Swerve, musicSelector));
        estop.whileTrue(new MusicPlayer(s_Swerve, musicSelector));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
