package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    // Set up tilt and twist motors, and encoders
    private final SparkFlex tiltMasterMotor = new SparkFlex(ArmConstants.kTiltMaster, MotorType.kBrushless);
    private final SparkFlex tiltSlaveMotor = new SparkFlex(ArmConstants.kTiltSlave, MotorType.kBrushless);
    private final SparkFlex extendMotor = new SparkFlex(ArmConstants.kExtend, MotorType.kBrushless);
    private final RelativeEncoder  tiltEncoder = tiltMasterMotor.getExternalEncoder();
    private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(ArmConstants.kPotentiometerPort);

    private SparkFlexConfig motorConfig = new SparkFlexConfig();
    private SparkFlexConfig slaveMotorConfig = new SparkFlexConfig();

    private double extendStartingPos;

    public Arm() {
        // Set yp slave config
        slaveMotorConfig.follow(ArmConstants.kTiltMaster);

        // Configure motors
        tiltMasterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        tiltSlaveMotor.configure(slaveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        extendMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Zero out encoder to start
        tiltEncoder.setPosition(0);

        extendStartingPos = getExtendValue();
    }

    public double getTiltEncoder() {
        return tiltEncoder.getPosition();
    }

    public double getExtendValue() {
        return potentiometer.get();
    }

    public void setTiltSpeed(double speed) {
        boolean positiveTilt = speed > 0 && getTiltEncoder() > ArmConstants.kMaxPositiveTilt;
        boolean negativeTilt = speed < 0 && getTiltEncoder() < ArmConstants.kMaxNegativeTilt;

        if (positiveTilt && negativeTilt) {
        tiltMasterMotor.set(ArmConstants.kMaxTiltSpeed * MathUtil.clamp(speed, -1, 1));
        } else {
            tiltMasterMotor.set(0);

        }
    }

    public void setExtendSpeed(double speed) {
        if ((speed > 0 && getExtendValue() < ArmConstants.kMaxExtend) || (speed < 0 && getExtendValue() > extendStartingPos)) {
            extendMotor.set(ArmConstants.kMaxExtendSpeed * MathUtil.clamp(speed, -1, 1));
        } else {
            extendMotor.set(0);
        }
    }

    // Returns the maximum inches allowed for the length of the arm from 
    // the base of the gearbox to the edge of the effector
    public double getMaxDistance(){
        // angle = a (getTiltEncoder()) / h (?)
        // h = a / angle
        return ArmConstants.kMaxDistanceFromFrame / (getTiltEncoder() / 360);
    }
}
