package frc.fridowpi.pneumatics;

import java.util.function.Supplier;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import frc.fridowpi.initializer.Initializer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticHandler implements IPneumaticHandler {

    private static IPneumaticHandler instance = null;
    private boolean initialized = false;
    private static Supplier<IPneumaticHandler> factory = PneumaticHandler::new;

    private int compressorId;
    private PneumaticsModuleType pcmType;

    private static final Logger logger = LogManager.getLogger(PneumaticHandler.class);

    private Compressor compressor;

    private PneumaticHandler() {
        Initializer.getInstance().addInitialisable(this);
    }

    public static IPneumaticHandler getInstance() {
        if (instance == null) {
            instance = factory.get();
        }
        return instance;
    }

    public static void setFactory(Supplier<IPneumaticHandler> factory) {
        PneumaticHandler.factory = factory;
        if (instance != null) {
            logger.warn("setFactory does nothing, instance has already been created");
        }
    }

    public static void reset() {
        instance = null;
        logger.info("PneumaticHandler is reset");
    }

    @Override
    public void enableCompressor() {
        compressor.enableDigital();
    }

    @Override
    public void disableCompressor() {
        compressor.disable();
    }

    @Override
    public void configureCompressor(int id, PneumaticsModuleType pcmType) {
        compressorId = id;
        this.pcmType = pcmType;
    }
    
    public boolean isCompressorPumping() {
        return compressor.isEnabled();
    }

    @Override
    public int getCompressorId() {
        return compressorId;
    }

    @Override
    public PneumaticsModuleType getCompressorType() {
        return pcmType;
    }

    @Override
    public void init() {
        initialized = true;

        compressor = new Compressor(compressorId, pcmType);

        SendableRegistry.addLW(this, "PneumaticHandler");
    }

    @Override
    public boolean isInitialized() {
        return initialized;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        compressor.initSendable(builder);
    }

}
