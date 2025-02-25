package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunArm extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public double tiltSpeed, extendSpeed;

    public RunArm(double tiltSpeed, double extendSpeed) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.effector);

        this.tiltSpeed = tiltSpeed;
        this.extendSpeed = extendSpeed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Send speeds 
        RobotContainer.arm.setTiltSpeed(tiltSpeed);
        RobotContainer.arm.setExtendSpeed(extendSpeed);
    }
}