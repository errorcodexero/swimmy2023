package com.swervedrivespecialties.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

@FunctionalInterface
public interface SteerControllerFactory<Controller extends SteerController, SC> {
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller
    ) {
        container.addNumber("Current Angle", () -> Math.toDegrees(controller.getStateAngle()));
        container.addNumber("Target Angle", () -> Math.toDegrees(controller.getReferenceAngle()));
    }

    default Controller create(
            ShuffleboardContainer dashboardContainer,
            SC steerConfiguration,
            String canbus,
            MechanicalConfiguration mechConfiguration
    ) {
        var controller = create(steerConfiguration, canbus, mechConfiguration);
        addDashboardEntries(dashboardContainer, controller);

        return controller;
    }

    default Controller create(
            ShuffleboardContainer dashboardContainer,
            SC steerConfiguration,
            MechanicalConfiguration mechConfiguration
    ) {
        var controller = create(steerConfiguration, mechConfiguration);
        addDashboardEntries(dashboardContainer, controller);

        return controller;
    }

    default Controller create(
            SC steerConfiguration, 
            MechanicalConfiguration mechConfiguration
    ) {
        return create(steerConfiguration, "", mechConfiguration);
    }

    Controller create(SC steerConfiguration, String canbus, MechanicalConfiguration mechConfiguration);
}
