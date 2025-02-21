package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.WristConstants;

public class RunWrist extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public double tiltEncoderStartPos, twistEncoderStartPos, tiltTemp, twistTemp;

    public RunWrist() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.effector);
    }

    @Override
    public void initialize() {
        // Set starting positions from encoders
        tiltEncoderStartPos = RobotContainer.wrist.getTiltEncoder();
        twistEncoderStartPos = RobotContainer.wrist.getTwistEncoder();
    }

    @Override
    public void execute() {
        // Used to set twist speed to 0 so that it doesn't go past 90 degrees
        twistTemp = RobotContainer.m_operatorController.getRawAxis(1);
        tiltTemp = RobotContainer.m_operatorController.getRawAxis(5);

        RobotContainer.wrist.setTiltSpeed(twistTemp * WristConstants.kMaxTwistSpeed);
        RobotContainer.wrist.setTwistSpeed(tiltTemp * WristConstants.kMaxTiltSpeed);
    }
}