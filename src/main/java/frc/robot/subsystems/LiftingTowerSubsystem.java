package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
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

        motorMaster.setIdleMode(IdleMode.kBrake);
        motorSlave.setIdleMode(IdleMode.kBrake);
        motorMaster.setInverted(true);

        SparkMaxConfig limitConfig = new SparkMaxConfig();
        limitConfig.softLimit
        .forwardSoftLimit(Constants.LiftingTower.softLimitTopPos).forwardSoftLimitEnabled(true);

        motorSlave.follow(motorMaster, DirectionType.invertMaster);

        motorMaster.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        motorSlave.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters,
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("masterTicks", motorMaster::getEncoderTicks, null);
        builder.addDoubleProperty("masterVel", motorMaster::getEncoderVelocity, null);
        builder.addBooleanProperty("masterRevSwitch", this::isBottomSwitchPressed, null);
    }
}
