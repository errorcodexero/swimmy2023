package frc.robot.subsystems.oi;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.base.subsystems.oi.SwerveDriveGamepad;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class Swimmy2023OISubsystem extends OISubsystem {

    private final static String OIHIDIndexName = "panel:index";

    private final static SwerveDriveGamepad.SwerveButton[] resetButtons = { SwerveDriveGamepad.SwerveButton.Y, SwerveDriveGamepad.SwerveButton.B} ;
    private final static SwerveDriveGamepad.SwerveButton[] xActionButtons = { SwerveDriveGamepad.SwerveButton.RBack} ;

    private Swimmy2023OIDeviceHollister oipanel_ ;

    public Swimmy2023OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, "swimmy2023oi", GamePadType.Swerve, db, true, true);

        int index ;
        MessageLogger logger = getRobot().getMessageLogger() ;

        //
        // Add the custom OI for zeke to the OI subsystem
        //
        try {
            index = getSettingsValue(OIHIDIndexName).getInteger() ;
        } catch (BadParameterTypeException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" exists, but is not an integer").endMessage();
            index = -1 ;
        } catch (MissingParameterException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" does not exist").endMessage();
            index = -1 ;            
        }

        if (index != -1) {
            try {
                oipanel_ = new Swimmy2023OIDeviceHollister(this, "OI", index) ;
                addHIDDevice(oipanel_) ;
            }
            catch(Exception ex) {
                logger.startMessage(MessageType.Error) ;
                logger.add("OI HID device was not created - ") ;
                logger.add(ex.getMessage()).endMessage(); ;
            }
        }         
    }

    @Override
    protected void gamePadCreated(Gamepad gp) {
        SwerveDriveGamepad swgp = (SwerveDriveGamepad)gp ;
        if (swgp != null) {
            swgp.setSwerveResetButtons(resetButtons);
            swgp.setSwerveXButtons(xActionButtons);
        }
    }

    public boolean isActionButtonPressed() {
        return oipanel_.isActionButtonPressed();
    }

    public boolean isDropButtonPressed() {
        return oipanel_.isDropButtonPressed();
    }
}
