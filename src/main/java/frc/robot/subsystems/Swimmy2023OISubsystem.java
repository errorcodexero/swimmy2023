package frc.robot.subsystems;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;

public class Swimmy2023OISubsystem extends org.xero1425.base.subsystems.oi.OISubsystem {

    public Swimmy2023OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, "swimmy2023oi", GamePadType.Swerve, db, true);

        SwerveDriveGamepad gp = (SwerveDriveGamepad)getGamePad() ;
        if (gp != null) {
            gp.setSwerveResetButtons( 
                new SwerveDriveGamepad.SwerveResetButton[] 
                    { SwerveDriveGamepad.SwerveResetButton.Y, SwerveDriveGamepad.SwerveResetButton.B }) ;
        }
    }
}
