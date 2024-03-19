package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    private final JoystickButton setArmPosition = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton reverseIntake = new JoystickButton(operator, XboxController.Button.kB.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Photonvision s_Photonvision = new Photonvision(
                                                    "Apriltag Camera",
                                                    25,
                                                    1.45,
                                                    15
                                                    );
    private final Arm s_Arm = new Arm();
    private final Intake s_Intake = new Intake();
    private final Outtake s_Outtake = new Outtake();

    /* Sendable Choosers */
    private final SendableChooser<String> musicSelector = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        NamedCommands.registerCommand("launchNote", new LaunchNote(s_Swerve, s_Arm, s_Outtake, s_Intake, s_Photonvision, () -> 4500));
        NamedCommands.registerCommand("startIntake", new TeleopIntake(s_Intake, () -> 4000, true));
        NamedCommands.registerCommand("stopIntake", new TeleopIntake(s_Intake, () -> 0, true));
        NamedCommands.registerCommand("lowerArm", new SetArmPosition(s_Arm, () -> 23));

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
        SmartDashboard.putNumber("targetHeight", 84);

        SmartDashboard.putNumber("armP", 0);
        SmartDashboard.putNumber("armI", 0);
        SmartDashboard.putNumber("armD", 0);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
                () -> -driver.getRawAxis(strafeAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
                () -> -driver.getRawAxis(rotationAxis) * 0.60 * SmartDashboard.getNumber("SpeedLimit", 1),
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Arm.setDefaultCommand(
            new TeleopArm(
                s_Arm,
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
                () -> operator.getRawAxis(outtakeAxis) * SmartDashboard.getNumber("ShooterSpeed", 1)
            )
        );

        // Configure the button bindings
        configureButtonBindings();
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
                                    s_Swerve,
                                    s_Arm,
                                    s_Outtake,
                                    s_Intake,
                                    s_Photonvision,
                                    () -> SmartDashboard.getNumber("Launcher set velocity", 4500)
                                    )
                                );
        reverseIntake.whileTrue(new TeleopIntake(s_Intake, () -> -0.25, false));
        reverseIntake.whileTrue(new TeleopOuttake(s_Outtake, () -> -0.25));
        resetWheels.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        setArmPosition.whileTrue(new SetArmPosition(s_Arm, () -> 90));
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
