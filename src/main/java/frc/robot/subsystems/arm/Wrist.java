package frc.robot.subsystems.arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
    // Set up tilt and twist motors, and encoders
    private final SparkMax tiltMotor = new SparkMax(WristConstants.kTilt, MotorType.kBrushless);
    private final SparkMax twistMotor = new SparkMax(WristConstants.kTwist, MotorType.kBrushless);
    private final RelativeEncoder tiltEncoder = tiltMotor.getAlternateEncoder();
    private final RelativeEncoder twistEncoder = twistMotor.getAlternateEncoder();

    private SparkMaxConfig motorConfig = new SparkMaxConfig();;

    private double tiltEncoderStartPos, twistEncoderStartPos;

    public Wrist() {
        tiltMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        twistMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        // Set starting positions from encoders
        tiltEncoderStartPos = RobotContainer.wrist.getTiltEncoder();
        twistEncoderStartPos = RobotContainer.wrist.getTwistEncoder();
    }

    public double getTwistEncoder() {
        return twistEncoder.getPosition();
    }

    public double getTiltEncoder() {
        return tiltEncoder.getPosition();
    }

    public void setTiltSpeed(double speed) {

        boolean positiveTilt = speed > 0 && tiltEncoderStartPos + getTiltEncoder() > WristConstants.kMaxPositiveTilt;
        boolean negativeTilt = speed < 0 && tiltEncoderStartPos - getTiltEncoder() < WristConstants.kMaxNegativeTilt;

        if (positiveTilt && negativeTilt) {
            tiltMotor.set(WristConstants.kMaxTiltSpeed * MathUtil.clamp(speed, -1, 1));
        } else {
            tiltMotor.set(0);
        }
    }

    public void setTwistSpeed(double speed) {

        boolean positiveTwist = speed > 0 && twistEncoderStartPos + getTwistEncoder() > WristConstants.kMaxPositiveTwist;
        boolean negativeTwist = speed < 0 && twistEncoderStartPos - getTwistEncoder() < WristConstants.kMaxNegativeTwist;

        if (positiveTwist && negativeTwist) { 
            twistMotor.set(WristConstants.kMaxTwistSpeed * MathUtil.clamp(speed, -1, 1));
        } else {
            twistMotor.set(0);
        }

    }
}