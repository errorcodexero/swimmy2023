package org.xero1425.base.controllers;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.DriverStation;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

/// \file

/// \brief The base class for a robot specific Auto Mode Controller
public abstract class AutoController extends BaseController {
    
    // The current automode
    private AutoMode current_automode_ ;

    // If true, we are in the test mode
    private boolean test_mode_ ;

    // If true, the automode has been started
    private boolean started_ ;

    // The list of automodes available
    private List<AutoMode> automodes_ ;

    // The settings file key that is used to enable the test mode
    private static final String testmode = "testmode:enabled";

    /// \brief Create a new AutoController object
    /// \param robot the robot objectg
    /// \param name the name of the autmode controller
    public AutoController(XeroRobot robot, String name) throws MissingParameterException, BadParameterTypeException {
        super(robot, name) ;

        automodes_ = new ArrayList<AutoMode>() ;

        // Check the settings file to see if the settings file is requesting test mode
        ISettingsSupplier settings = robot.getSettingsSupplier() ;
        if (settings.isDefined(testmode)) {
            SettingsValue v = settings.get(testmode) ;
            if (v.isBoolean() && v.getBoolean()) {
                test_mode_ = true ;
            }
        }
    }

    /// \brief Add a new automode to the automode controller
    /// \param mode the automode to add to the set
    public void addAutoMode(AutoMode mode) {
        automodes_.add(mode) ;
    }

    /// \brief initialize the automode
    @Override
    public void init() {
    }

    /// \brief Returns a list of all automodes available
    /// \returns a list of all automodes available
    public List<AutoMode> getAllAutomodes()  {
        return automodes_ ;
    }

    /// \brief Called once per robot loop to run the automode.
    @Override
    public void run() {
        if (current_automode_ != null) {
            if (!started_) {
                try {
                    current_automode_.start() ;
                    started_ = true ;
                }
                catch(Exception ex) {
                    MessageLogger logger = getRobot().getMessageLogger();
                    logger.startMessage(MessageType.Error) ;
                    logger.add("exception thrown in start() method of automode - ").add(ex.getMessage()) ;
                    logger.endMessage();
                }
            }
            try {
                if (!current_automode_.isDone())
                    current_automode_.run() ;
            }
            catch(Exception ex) {
                ex.printStackTrace();
                MessageLogger logger = getRobot().getMessageLogger();
                logger.startMessage(MessageType.Error) ;
                logger.add("exception thrown in run() method of automode - ").add(ex.getStackTrace().toString()) ;
                logger.endMessage();            
            }
        }
    }

    /// \brief Returns true if we are in the test mode.  The test mode is triggered if the
    /// set mode setting "testmode:enabled" is true and the robot is NOT connected to an FMS system.
    /// \returns true if we are in the test mode
    public boolean isTestMode() {
        if (DriverStation.isFMSAttached())
            return false ;

        return test_mode_ ;
    }

    /// \brief Update the automodes if necessary.  This is expected to be overridden by the
    /// derived class to perform the desired functions.
    /// \param mode the autumode number read from the automode selector switch on the OI
    /// \param gamedata the game data read from the WPILib driver station APIs.
    public void updateAutoMode(int mode, String gamedata) throws Exception {
        setAutoMode(null) ;
    }

    /// \brief Return the current automode name, or NONE if there is not selected automode
    /// \returns the current automode name, or NONE if there is not selected automode
    public String getAutoModeName() {
        if (current_automode_ == null)
            return "NONE" ;

        return current_automode_.getName() ;
    }

    /// \brief Return the current automode
    /// \returns the current automode
    public AutoMode getAutoMode() {
        return current_automode_ ;
    }

    /// \brief Set the current automode
    /// \param mode the automode to make current
    protected void setAutoMode(AutoMode mode) {
        current_automode_ = mode ;
        started_ = false ;
    }
} ;