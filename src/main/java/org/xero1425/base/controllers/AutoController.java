package org.xero1425.base.controllers;

import java.util.ArrayList;
import java.util.List;
import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief The base class for a robot specific Auto Mode Controller
public abstract class AutoController extends BaseController {
    
    // The current automode
    private AutoMode current_automode_ ;

    // If true, the automode has been started
    private boolean started_ ;

    // The list of automodes available
    private List<AutoMode> automodes_ ;
    private TestAutoMode test_mode_ ;

    private int test_mode_number_ ;

    /// \brief Create a new AutoController object
    /// \param robot the robot objectg
    /// \param name the name of the autmode controller
    public AutoController(XeroRobot robot, String name) throws MissingParameterException, BadParameterTypeException {
        super(robot, name) ;

        automodes_ = new ArrayList<AutoMode>() ;
        test_mode_number_ = Integer.MAX_VALUE ;
    }

    public void setTestMode(TestAutoMode mode) {
        test_mode_ = mode ;
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

    /// \brief Update the automodes if necessary.  This is expected to be overridden by the
    /// derived class to perform the desired functions.
    /// \param mode the autumode number read from the automode selector switch on the OI
    /// \param gamedata the game data read from the WPILib driver station APIs.
    public boolean updateAutoMode(int mode, String gamedata) throws Exception {
        boolean b = true ;

        if (mode < 0) {
            //
            // This is a test mode
            //
            if (current_automode_ != test_mode_ || test_mode_number_ != mode) {
                current_automode_ = test_mode_ ;
                test_mode_number_ = mode ;
                
                try {
                    test_mode_.setTestModeTestNumber(-mode) ;
                }
                catch(Exception ex) {
                    current_automode_ = null ;
                    b = false ;
                }
            }
        }
        else if (mode == 0) {
            //
            // We do nothing
            //
            current_automode_ = null ;
        }
        else {
            //
            // We run a real automode
            //
            mode-- ;
            if (mode >= automodes_.size()) {
                //
                // Somehow these are a problem
                //
                current_automode_ = null ;
                b = false ;
            }
            else {
                current_automode_ = automodes_.get(mode) ;
            }
        }

        return b ;
    }

    /// \brief Return the current automode name, or NONE if there is not selected automode
    /// \returns the current automode name, or NONE if there is not selected automode
    public String getAutoModeName() {
        String ret = "NONE" ;

        if (current_automode_ == test_mode_) {
            ret = test_mode_.getCurrentModeName() ;
        }
        else if (current_automode_ != null) {
            ret = current_automode_.getName() ;
        }

        return ret ;
    }

    /// \brief Return the current automode
    /// \returns the current automode
    public AutoMode getAutoMode() {
        return current_automode_ ;
    }

    public TestAutoMode getTestAutoMode() {
        return test_mode_ ;
    }

    /// \brief Set the current automode
    /// \param mode the automode to make current
    protected void setAutoMode(AutoMode mode) {
        current_automode_ = mode ;
        started_ = false ;
    }
} ;