package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunWrist extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public double tiltSpeed, twistSpeed;

    public RunWrist(double tiltSpeed, double twistSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.effector);

        this.tiltSpeed = tiltSpeed;
        this.twistSpeed = twistSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Send speeds 
        RobotContainer.wrist.setTiltSpeed(tiltSpeed);
        RobotContainer.wrist.setTwistSpeed(twistSpeed);
    }
}