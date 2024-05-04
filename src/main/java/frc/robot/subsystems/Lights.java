package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    private final PowerDistribution pdh = new PowerDistribution();

    private final Swerve swerve;
    private final Arm arm;
    private final Intake intake;
    private final Outtake outtake;

    public Lights(Swerve swerve, Arm arm, Intake intake, Outtake outtake) {
        // Runs when calling new Template()
        this.swerve = swerve;
        this.arm = arm;
        this.intake = intake;
        this.outtake = outtake;
        pdh.setSwitchableChannel(true);
    }

    @Override
    public void periodic() {
        // Stuff to run repeatedly
        if(swerve.rotationAtSetpoint && arm.atSetpoint && outtake.atSpeed && !intake.getSecondaryNoteSensor().get()) {
            pdh.setSwitchableChannel(!pdh.getSwitchableChannel());
        } else if(!intake.getNoteSensor().get() || !intake.getSecondaryNoteSensor().get()) {
            pdh.setSwitchableChannel(true);
        } else {
            pdh.setSwitchableChannel(false);
        }
    }
}