package com.swervedrivespecialties.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface DriveControllerFactory<Controller extends DriveController, DriveConfiguration> {
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller
    ) {
        container.addNumber("Current Velocity", controller::getStateVelocity);
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            String canbus,
            MechanicalConfiguration mechConfiguration
    ) {
        var controller = create(driveConfiguration, canbus, mechConfiguration);
        addDashboardEntries(container, controller);

        return controller;
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            MechanicalConfiguration mechConfiguration
    ) {
        var controller = create(driveConfiguration, mechConfiguration);
        addDashboardEntries(container, controller);

        return controller;
    }

    default Controller create(
        DriveConfiguration driveConfiguration,
        MechanicalConfiguration mechConfiguration
    ) {
        return create(driveConfiguration, "", mechConfiguration);
    }

    Controller create(DriveConfiguration driveConfiguration, String canbus, MechanicalConfiguration mechConfiguration);
}
