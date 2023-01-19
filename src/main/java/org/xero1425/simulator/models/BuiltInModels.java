package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.ModelFactory;

public class BuiltInModels {
    private BuiltInModels() {
    }

    static public void registerBuiltinModels(ModelFactory factory) {
        factory.registerModel("navx", "org.xero1425.simulator.models.NavXModel");
        factory.registerModel("fms", "org.xero1425.simulator.models.FMSModel");
        factory.registerModel("limelight", "org.xero1425.simulator.models.LimeLightModel");
        factory.registerModel("drivergamepad", "org.xero1425.simulator.models.DriverGamepadModel");
        factory.registerModel("tankdrive", "org.xero1425.simulator.models.TankDriveModel");
        factory.registerModel("swervedrive", "org.xero1425.simulator.models.SwerveDriveModel");
        factory.registerModel("motor-encoder", "org.xero1425.simulator.models.MotorEncoderSubsystemModel");
    }
}
