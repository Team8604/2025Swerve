package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunWrist extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public double tiltEncoderStart, twistEncoderStart;

    public RunWrist(double tiltTarget, double twistTarget) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.effector);
    }

    @Override
    public void initialize() {
        // Set starting positions from encoders
        tiltEncoderStart = RobotContainer.wrist.getTiltEncoder();
        twistEncoderStart = RobotContainer.wrist.getTwistEncoder();
    }

    @Override
    public void execute() {
        // TEMPORARY
        RobotContainer.wrist.setTiltSpeed(0);
        RobotContainer.wrist.setTwistSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // TEMPORARY
        return false;
    }
}