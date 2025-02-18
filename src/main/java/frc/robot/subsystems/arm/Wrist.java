package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    // Set up tilt and twist motors, and encoders
    private final SparkMax tiltMotor = new SparkMax(WristConstants.kTilt, MotorType.kBrushless);
    private final SparkMax twistMotor = new SparkMax(WristConstants.kTwist, MotorType.kBrushless);
    private final DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(WristConstants.kTiltEncoder);
    private final DutyCycleEncoder twistEncoder = new DutyCycleEncoder(WristConstants.kTwistEncoder);

    private SparkMaxConfig motorConfig = new SparkMaxConfig();;

    public Wrist() {
        tiltMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        twistMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setTiltSpeed(double speed) {
        tiltMotor.set(WristConstants.kMaxTiltSpeed * MathUtil.clamp(speed, -1, 1));
    }

    public void setTwistSpeed(double speed) {
        twistMotor.set(WristConstants.kMaxTwistSpeed * MathUtil.clamp(speed, -1, 1));
    }

    public double getTwistEncoder() {
        return twistEncoder.get();
    }

    public double getTiltEncoder() {
        return tiltEncoder.get();
    }
}