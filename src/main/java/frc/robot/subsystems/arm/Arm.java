package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;

public class Arm extends SubsystemBase {
    // Set up tilt and twist motors, and encoders
    private final SparkFlex tiltMasterMotor = new SparkFlex(ArmConstants.kTiltMaster, MotorType.kBrushless);
    private final SparkFlex tiltSlaveMotor = new SparkFlex(ArmConstants.kTiltSlave, MotorType.kBrushless);
    private final SparkFlex extendMotor = new SparkFlex(ArmConstants.kExtend, MotorType.kBrushless);
    private final RelativeEncoder  tiltEncoder = tiltMasterMotor.getExternalEncoder();

    private SparkFlexConfig motorConfig = new SparkFlexConfig();


    public Arm() {
        // Configure motors
        tiltMasterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        tiltSlaveMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        extendMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        tiltSlaveMotor.follow(tiltMasterMotor, true); //FIX error later

        // Zero out encoder to start
        tiltEncoder.setPosition(0);
    }

    public double getTiltEncoder() {
        return tiltEncoder.getPosition();
    }

    public void setTiltSpeed(double speed) {
        tiltMasterMotor.set(ArmConstants.kmaxTiltSpeed * MathUtil.clamp(speed, -1, 1));
    }

    public void setTwistSpeed(double speed) {
        extendMotor.set(ArmConstants.kmaxTiltSpeed * MathUtil.clamp(speed, -1, 1));
    }
}
