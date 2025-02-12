package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class LiftingTowerSubsystem extends SubsystemBase {
    private final PidValues pidValues = Constants.LiftingTower.pidValues; // p, i, d, f

    private SparkMaxConfig motorConfig;

    private FridoSparkMax motorSlave;
    private FridoSparkMax motorMaster;

    private MAXMotionConfig smartMotionConfig;

    public LiftingTowerSubsystem() {
        motorSlave = new FridoSparkMax(Constants.LiftingTower.liftingTowerRightId);
        motorMaster = new FridoSparkMax(Constants.LiftingTower.liftingTowerLeftId);

        motorSlave.follow(motorMaster, DirectionType.invertMaster);

        motorConfig = new SparkMaxConfig();
        smartMotionConfig = new MAXMotionConfig();

        motorMaster.enableReverseLimitSwitch(Constants.LiftingTower.towerBottomSwitchPolarity, true);

        motorConfig.closedLoop.p(pidValues.kP)
                .i(pidValues.kI)
                .d(pidValues.kD)
                .outputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward)
                .velocityFF(pidValues.kF.orElse(0.0));

        pidValues.iZone.ifPresent(iZone -> motorConfig.closedLoop.iZone(iZone));

        smartMotionConfig.allowedClosedLoopError(Constants.LiftingTower.kAllowedClosedLoopError);
        smartMotionConfig.maxAcceleration(Constants.LiftingTower.kMaxAcceleration);
        smartMotionConfig.maxVelocity(Constants.LiftingTower.kMaxVelocity);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);

        motorMaster.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void resetEncoder() {
        motorMaster.setEncoderPosition(Constants.LiftingTower.resetEncoderPosition);
    }

    public boolean isBottomSwitchPressed() {
        return motorMaster.isReverseLimitSwitchActive();
    }

    public void setPercent(double percent) {
        motorMaster.set(Math.max(Constants.LiftingTower.zeroingSpeed, percent));
    }

    @Override
    public void periodic() {
        if (isBottomSwitchPressed()) {
            resetEncoder();
        }
    }

    public void setMotorSpeed(double speed) {
        motorMaster.set(speed);
    }

    public void setHeight(double position) {
        motorMaster.setPosition(
                Math.max(0.0,
                        Math.min(
                                position,
                                Constants.LiftingTower.softLimitTopPos)));
    }

    public void stopMotors() {
        motorMaster.stopMotor();
    }
}
