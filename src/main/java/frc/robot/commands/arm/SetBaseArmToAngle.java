package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class SetBaseArmToAngle extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    PIDController extendPid;
    PIDController tiltPid;

    double extendTarget, tiltTarget;

    public SetBaseArmToAngle(double extendTarget, double tiltTarget) {
        this.extendTarget = extendTarget;
        this.tiltTarget = tiltTarget;

        extendPid = new PIDController(ArmConstants.kExtendP, ArmConstants.kExtendI, ArmConstants.kExtendD);
        tiltPid = new PIDController(ArmConstants.kTiltP, ArmConstants.kTiltI, ArmConstants.kTiltD);
    }

    @Override
    public void execute() {
        RobotContainer.arm.setExtendSpeed(extendPid.calculate(RobotContainer.arm.getExtendValue(), extendTarget));
        RobotContainer.arm.setTiltSpeed(tiltPid.calculate(RobotContainer.arm.getTiltEncoder(), tiltTarget));
    }

    @Override
    public boolean isFinished() {
        if (extendPid.atSetpoint() && tiltPid.atSetpoint()){
            return true;
        } else {
            return false;
        }
    }
}
