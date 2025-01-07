package frc.fridowpi.sensors;


import frc.fridowpi.initializer.Initializer;
import frc.fridowpi.sensors.base.INavx;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.util.function.Function;

public class FridoNavx extends AHRS implements INavx {
    private static Function<SPI.Port, INavx> factory = FridoNavx::new;

    private static SPI.Port port;

    public static void setup(SPI.Port port) {
        FridoNavx.port = port;
    }

    private static float yawOffset = 0.f;


    private static float pitchOffset = 0.f;
    private static float rollOffset = 0.f;

    /**
     * @return offset in degrees
     */
    public static float getYawOffset() {
        return yawOffset;
    }

    /**
     * @return offset in degrees
     */
    public static float getPitchOffset() {
        return pitchOffset;
    }

    /**
     * @return offset in degrees
     */
    public static float getRollOffset() {
        return rollOffset;
    }

    /**
     * @param offset in degrees
     */
    public static void setYawOffset(float offset) {
        yawOffset = offset;
    }

    /**
     * @param offset in degrees
     */
    public static void setPitchOffset(float offset) {
        pitchOffset = offset;
    }

    /**
     * @param offset in degrees
     */
    public static void setRollOffset(float offset) {
        rollOffset = offset;
    }

    /**
     * @param offset in degrees
     */
    public static void setYawOffset(double offset) {
        yawOffset = (float) offset;
    }

    /**
     * @param offset in degrees
     */
    public static void setPitchOffset(double offset) {
        pitchOffset = (float) offset;
    }

    /**
     * @param offset in degrees
     */
    public static void setRollOffset(double offset) {
        rollOffset = (float) offset;
    }

    public static void setFactory(Function<SPI.Port, INavx> factory) {
        FridoNavx.factory = factory;
    }

    private FridoNavx(SPI.Port port) {
        super(port);
        Initializer.getInstance().addInitialisable(this);
    }

    private static INavx instance;

    public static INavx getInstance() {
        if (instance == null) {
            logger.info("created new instance");
            instance = factory.apply(port);
        }
        return instance;
    }

    @Override
    public float getYaw() {
        return super.getYaw() + yawOffset;
    }

    @Override
    public float getPitch() {
        return super.getPitch() + pitchOffset;
    }

    @Override
    public float getRoll() {
        return super.getRoll() + rollOffset;
    }

    @Override
    public double getAngle() {
        return super.getAngle() + yawOffset;
    }


    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(super.getRotation2d().getDegrees() + yawOffset);
    }

    private static Logger logger = LogManager.getLogger(FridoNavx.class);
    private boolean initialized = false;

    @Override
    public void init() {
        logger.info("DONE Navix initialized");
        logger.info("resetting Navx.");
        reset();

        initialized = true;
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }
}
