package frc.robot.subsystems;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;

public class Swimmy2023OISubsystem extends org.xero1425.base.subsystems.oi.OISubsystem {

    public Swimmy2023OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, "swimmy2023oi", GamePadType.Swerve, db, true);
    }
}
