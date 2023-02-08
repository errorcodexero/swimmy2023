package org.xero1425.base.subsystems;

import java.util.List;
import java.text.DecimalFormat;
import java.util.ArrayList;

import org.xero1425.base.LoopType;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/// \file

/// \brief The base class for all subsystems that make up the robot.
/// The subsystem class manages the interaction of subsytems with the robot control loop for each
/// subsystem.  A subsystem goes through a specific lifecycle.
/// \li \c construction - the constructor is called to create the subsystem
/// \li \c computeMyState() - this is called to compute the initial state of the subsystem
/// \li \c postHWInit() - called once to do any initialization where state needs to be known
/// \li \c reset() - called during initialization as the robot enters the disabled state
/// \li \c init() - called to initialize hardware for the start of auto
/// \li \c computeMyState() - called during the robot loop once per loop during auto
/// \li \c run() - called during the robot loop once per loop during auto
/// \li \c reset() - called after auto as the robot enters the disabled state between auto and teleop
/// \li \c init() - called to initialize hardware for the start of teleop
/// \li \c computeMyState() - called during the robot loop once per loop during teleop
/// \li \c run() - called during the robot loop once per loop during teleop
/// \li \c reset() - called after teleop as the robot enters the disabled state
///
public class Subsystem {
    //
    // If true, we turn on subsystem timing
    //
    static private boolean timing_ = true ;

    //
    // The current action assigned to this subsystme
    //
    private Action action_;

    //
    // The default action to run when no action is assigned
    //
    private Action default_action_;

    //
    // THe parent subsystem of this subsystem
    //
    private final Subsystem parent_;

    //
    // The name of this subsystem
    //
    private final String name_;

    //
    // The children subsystem of this subsystem
    //
    private final List<Subsystem> children_;

    //
    // The main robot object
    //
    private XeroRobot robot_;

    //
    // The ID for logging subsystem related messages to the log file
    //
    private final int logger_id_ ;

    //
    // If true, the default action actually finished (we don't really want this to happen)
    //
    private boolean finished_default_ ;

    //
    // The total time spent running code in this subsystem.  This captures the compute state
    // run time since this is the method that takes a lot of time in time critical subsystem code.
    //
    private double total_time_ ;

    //
    // The number of times this subsystem has been run
    //
    private int total_cnt_ ;

    //
    // The maximum time spent on any one robot loop in the code for this subsystem
    //
    private double max_time_ ;

    //
    // The output format for logging information about the subsystem run times
    //
    private DecimalFormat fmt_ ;

    //
    // The minimum time spend on any one robot loop in the code for this subsystem
    //
    private double min_time_ ;

    //
    // If true, this subsystem logs much information
    //
    private boolean verbose_ ;

    /// \brief used to give the display type for a value
    public enum DisplayType {
        Always,                 ///< Always display this value
        Verbose,                ///< Display this value when running
        Disabled,               ///< Display only when the robot is disabled
    } ;

    private Subsystem(XeroRobot robot, final Subsystem parent, final String name) {
        name_ = name;
        parent_ = parent;
        children_ = new ArrayList<Subsystem>();
        robot_ = robot ;

        action_ = null;
        default_action_ = null;

        logger_id_ = getRobot().getMessageLogger().registerSubsystem(name);

        finished_default_ = false;
        verbose_ = false;

        if (timing_)
        {
            total_time_ = 0.0;
            total_cnt_ = 0;
            min_time_ = Double.MAX_VALUE;
            max_time_ = 0.0;
            fmt_ = new DecimalFormat("00.000");
        }
        try {
            ISettingsSupplier p = getRobot().getSettingsSupplier();
            String pname = "system:verbose:" + name_ ;
            if (p.isDefined(pname) && p.get(pname).isBoolean() && p.get(pname).getBoolean())
                verbose_ = true;
            
        } catch (MissingParameterException e) {
            // Should never happen
            verbose_ = false ;
        } catch (BadParameterTypeException e) {
            // Should never happen
            verbose_ = false ;
        }
    }

    /// \brief Create a new subsystem with no parent
    /// \param robot the robot this subsystem belongs to
    /// \param name the name of this subsystem
    public Subsystem(XeroRobot robot, final String name) {
        this(robot, null, name) ;
    }

    /// \brief Create a new subsystem
    /// \param parent the parent for the current subsystem
    /// \param name the name of the current subsystem
    public Subsystem(final Subsystem parent, final String name) {
        this(parent.getRobot(), parent, name) ;
    }

    public String getStatus() {
        return "This subsystem is not publishing status" ;
    }

    // public void publishStatus() {
    //     getRobot().publishSubsystemStatus(getName(), getStatus());
    //     publishChildSubsystems();
    // }

    // protected void publishChildSubsystems() {
    //     for(Subsystem sys : children_) {
    //         sys.publishStatus(); ;
    //     }
    // }

    /// \brief Return a settings file value associated with this subsystem
    ///
    /// This method searches for a settings value assocaited with the given subsystem.  Subsystem values
    /// are stored under the "subsystems" key in the settings file, and then under a key that is the same as the 
    /// name of the subsystem. So, if the subsystem is named "tankdrive", the getSettingsValue("width"), would 
    /// return the value of the setting stored at subsystems:tankedrive:width.
    ///
    ///     subsystems : {
    ///       tankdrive : {
    ///          width : 30.0,
    ///          length : 32.0,
    ///          inches_per_tick: 0.006
    ///       }        
    ///     } 
    ///
    /// So, as shown above, <em>getSettingsValue("width")</em> would return 30.0
    ///
    /// \param name the name of the setting associated with a subsystem
    /// \exception throws MissingParameterException if the setting is not present in the settings file
    /// \returns the SettingsValue for the given name
    public SettingsValue getSettingsValue(String name) throws MissingParameterException {
        return getRobot().getSettingsSupplier().get("subsystems:" + name_ + ":" + name) ;
    }

    /// \brief Returns true if the subsystem related setting is defined
    /// \returns true if the subsystem related setting is defined
    public boolean isSettingDefined(String name) {
        return getRobot().getSettingsSupplier().isDefined("subsystems:" + name_ + ":" + name) ;
    }

    /// \brief returns true if this is the OI subsystem
    /// \returns true if this is the OI subsystem
    public boolean isOI() {
        return false ;
    }
    
    /// \brief returns true if this is the drivebase subsystem
    /// \returns true if this is the drivebase subsystem
    public boolean isDB() {
        return false ;
    }

    /// \brief returns a property for the simulation system.
    /// This base class always returns null.  This method is expected to be overridden
    /// in a derived class to return subsystem specific properties.
    /// \param name the name of the property to return
    /// \returns always returns null
    public SettingsValue getProperty(String name) {
        return null ;
    }

    /// \brief returns a subsystem by name.
    /// If this subsystem has the requested name, this subsystem is returned.  Otherwise
    /// all of the children for the current subsystem are searched recursively until a
    /// subsystem with the name given is found.
    /// \param name the name of the subsystem desired
    /// \returns the subsystem with the given name, or null if it does not exist within this subsystem
    public Subsystem getSubsystemByName(String name) {
        if (name_.equals(name))
            return this ;

        return getChildByName(name) ;
    }

    /// \brief get a child subsystem by name
    /// All child subsystem are searched recursively until a subsystem with the name given
    /// is found.  If a subsystem with the given name is not found, null is returned
    /// \param name the name of the subsystem of interest
    /// \returns the subsystem with the given nama, or null if a subsystem with the name does not exist
    public Subsystem getChildByName(String name) {
        for(Subsystem child: children_) {
            Subsystem ret = child.getChildByName(name) ;
            if (ret != null)
                return ret ;

            if (child.getName().equals(name))
                return child ;
        }

        return null ;
    }

    /// \brief returns the name of the current subsystem
    /// \returns the name of the current subsystem
    public String getName() {
        return name_;
    }

    /// \brief returns the parent of the subsystem
    /// \returns the parent of the subsystem
    public Subsystem getParent() {
        return parent_;
    }

    /// \brief returns a reference to the robot object
    /// \returns a reference to the robot object
    public XeroRobot getRobot() {
        return robot_;
    }

    /// \brief return the logger ID for message assocaited with this subsystem
    /// \returns the logger iD for message associated with this subsystem
    public int getLoggerID() {
        return logger_id_ ;
    }

    /// \brief add a child subsystem to this current subystem
    /// \param sub the subsystem to add as a child
    public void addChild(final Subsystem sub) throws Exception {
        children_.add(sub);
    }

    /// \brief initialize the subsystem. 
    /// This is called at the start of the robot loop of each type.  In other words it is
    /// called at the start of autonomous (LoopType.Autonomous), teleop (LoopType.Teleop),
    /// and disabled (LoopType.Disabled).
    /// \param ltype the loop type we are running
    public void init(LoopType ltype) {
        for(Subsystem sub : children_)
            sub.init(ltype);        
    }

    /// \brief This method is called when the robot enters the disabled state.
    /// It is used to reset the hardware of the robot.  Keep in mind the robot enters
    /// the disabled state at the start of robot execution, between autonomous and
    /// teleop, and after teleop is complete.  This method can be used to reset hardware
    /// actions.  For instance, there was a turret rotating at the end of auto, this method
    /// could be used to reset the motors to stopped for the turret so they did not start
    /// up immediately when teleop begins.
    public void reset() {
        for(Subsystem sub : children_)
            sub.reset() ; 
    }

    /// \brief reservered for future use
    public void selfTest() {
    }

    /// \brief this method is called during hardware initialization.
    /// It is called after all of the subsystms are constructure and after computeState() has been
    /// called once so that the internal state of each subsystem is valid.
    public void postHWInit() throws Exception {
        for(Subsystem sub : children_)
            sub.postHWInit();
    }

    /// \brief this method computes the current state of the subsystem.
    /// This method is called when the robot is disabled, in auto mode, or in teleop mode.  This
    /// method calls subsystem specific computeMyState() which should be implemented by any
    /// derived class.  This specific implementation keeps track of execution time of the
    /// subsystem state computations as this is the most CPU intensive operation of the robot.
    /// Note, this method also catches all exceptions from the computeMyState() method keeping
    /// the exception from propogating up and crashing the robot code.
    ///
    public void computeState() {
        double start = 0.0 ;
        for(Subsystem sub : children_) {
            sub.computeState();
        }

        try {
            start = getRobot().getTime() ;

            computeMyState() ;
            
            if (timing_)
            {
                double elapsed = getRobot().getTime() - start ;
                total_time_ += elapsed ;
                total_cnt_++ ;

                min_time_ = Math.min(min_time_, elapsed) ;
                max_time_ = Math.max(max_time_, elapsed) ;
                
                // Turn this on to see where execution time is going
                
                if (total_cnt_ > 0 && (total_cnt_ % 50) == 0) {
                    MessageLogger logger = getRobot().getMessageLogger() ;
                    logger.startMessage(MessageType.Debug, getRobot().getLoggerID()) ;
                    logger.add("subsystem ").addQuoted(getName()) ;
                    logger.add("count", total_cnt_) ;
                    logger.add("min", fmt_.format(min_time_ * 1000)) ;
                    logger.add("average", fmt_.format(total_time_ / total_cnt_ * 1000)) ;
                    logger.add("max", fmt_.format(max_time_ * 1000)) ;
                    logger.endMessage();
                }
            }
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;            
            logger.startMessage(MessageType.Error) ;
            logger.add("subsystem ").addQuoted(getName()) ;
            logger.add(" threw exception in computeMyState() - ").add(ex.getMessage()) ;
            logger.endMessage();
            logger.logStackTrace(ex.getStackTrace());

        }
    }

    /// \brief this method is called in robot loop to set any hardware outputs
    /// This method is called in auto mode and teleop mode.  It takes the current state of the
    /// robot combined with the action assigned to the subsystem and determines the outputs
    /// required for actuators.
    ///
    public void run() throws Exception {
        if (action_ != null)
        {
            try {
                if (!action_.isDone()) {
                    action_.run() ;
                    if (action_.isDone()) {
                        if (action_ == default_action_)
                            finished_default_ = true ;
                        action_ = null ;
                    }
                }
            }
            catch(Exception ex) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("action ").addQuoted(action_.toString()) ;
                logger.add(" threw exception during run() - ").add(ex.getMessage()) ;
                logger.endMessage();
                logger.logStackTrace(ex.getStackTrace());
            }
        }

        //
        // If the current action is done, but there is a default action, and we
        // did not just complete the default action, then start the default action
        //
        if (action_ == null && default_action_ != null && !finished_default_)
        {
            try {
                cancelActionPlusChildren();
                action_ = default_action_ ;                
                action_.start() ;
                if (action_.isDone())
                    finished_default_ = true ;
            }
            catch(Exception ex) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("action ").addQuoted(action_.toString()) ;
                logger.add(" threw exception during start() - ").add(ex.getMessage()) ;
                logger.endMessage();
                getRobot().logStackTrace(ex.getStackTrace());
                action_ = null ;
            }
        }

        for(Subsystem sub : children_)
            sub.run();        
    }

    /// \brief set the current action for the subsystem
    /// \param act the action to set
    /// \returns true if the action was accepted, false if not
    public boolean setAction(final Action act) {
        return setAction(act, false) ;
    }

    /// \brief set the current action for the subsystem
    /// \param act the action to set
    /// \param parent_busy_ok ok to assign action if a parent is busy
    /// \returns true if the action was accepted, false if not
    public boolean setAction(final Action act, boolean parent_busy_ok) {
        if (act == default_action_ && act != null) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("action ").addQuoted(act.toString()) ;
            logger.add(" is being assigned in setAction() but is the default action") ;
            logger.endMessage();

            //
            // Trying to setAction with the same action that is already the
            // default action
            //
            return false ;
        }

        if (!parent_busy_ok && parent_.isAnyParentBusy())
        {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("action ").addQuoted(act.toString()) ;
            logger.add(" cannot be assigned because a parent is busy") ;
            logger.endMessage();            
            
            //
            // If we are busy the new action overrides the current action.  If any
            // parent is busy, we fail unless the parent_busy_ok flag is set.
            //
            return false ;
        }

        finished_default_ = false ;

        //
        // Cancel any current action for this subsystem or any action for any children
        // of this subsystem.
        //
        cancelActionPlusChildren();

        action_ = act ;
        try {
            //
            // Now start the new current action
            //
            if (action_ != null)
                action_.start() ;
        }
        catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("action ").addQuoted(action_.toString()) ;
            logger.add(" threw exception during start() - ").add(ex.getMessage()) ;
            logger.endMessage();
            getRobot().logStackTrace(ex.getStackTrace());
            action_ = null ;
        }

        return true ;
    }

    /// \brief returns the currently assigned action
    /// \returns the currently assigned action
    public Action getAction() {
        return action_;
    }

    /// \brief set the default action for the subsystem
    /// \param act the action to assign to the subsystem
    public void setDefaultAction(final Action act) {
        finished_default_ = false ;

        if (action_ != null && action_ == default_action_)
        {
            //
            // The running action is the default action, since we are 
            // replacing the default action, cancel this current action so that
            // we can start the new default action.
            //
            cancelActionPlusChildren();
            action_ = null ;
        }

        //
        // Store the default action
        //
        default_action_ = act ;
        if (action_ == null && default_action_ != null)
        {
            //
            // If nothing is running now, start the default action
            //
            try {
                cancelActionPlusChildren();
                action_ = default_action_ ;
                action_.start() ;
                if (action_.isDone())
                    finished_default_ = true ;
            }
            catch(Exception ex) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("action ").addQuoted(action_.toString()) ;
                logger.add(" threw exception during start() - ").add(ex.getMessage()) ;
                logger.endMessage();
                getRobot().logStackTrace(ex.getStackTrace());
            }
        }
    }

    /// \brief return the default action for the subsystem
    /// \returns the default action for the subsystem
    public Action getDefaultAction() {
        return default_action_;
    }

    /// \brief cancel the currently assigned action.
    /// Cancel requires that the action end immediately prior to this call returning
    public void cancelAction() {
        cancelAction(true) ;
    }

    /// \brief put a value on the driver station dashboard
    /// The dtype parameter controls when a value is displayed on the dashboard.  If dtype
    /// is set to DisplayType.Always, the value is always displayed on the dashboard.  If the
    /// dtype is set to Disabled, the value is only displayed when the robot is disabled.  This is
    /// useful for displaying values that aid in robot system checkes.  If the dtype is set to Verbose, 
    /// the value is only displayed when the robot is disabled or when the subsystem is in verbose 
    /// mode.  A subsystem is put in verbose mode, when the settings file contains a property with the
    /// name SUBSYSTEM:verbose set to true, where SUBSYSTEM is the name of the subsystem.
    /// \param name name of the value to display
    /// \param dtype indicates when the value should be displayed
    /// \param value the value to display
    public void putDashboard(String name, DisplayType dtype, boolean value) {
        if (shouldDisplay(dtype))
            SmartDashboard.putBoolean(name, value) ;
    }

    /// \brief put a value on the driver station dashboard
    /// The dtype parameter controls when a value is displayed on the dashboard.  If dtype
    /// is set to DisplayType.Always, the value is always displayed on the dashboard.  If the
    /// dtype is set to Disabled, the value is only displayed when the robot is disabled.  This is
    /// useful for displaying values that aid in robot system checkes.  If the dtype is set to Verbose, 
    /// the value is only displayed when the robot is disabled or when the subsystem is in verbose 
    /// mode.  A subsystem is put in verbose mode, when the settings file contains a property with the
    /// name SUBSYSTEM:verbose set to true, where SUBSYSTEM is the name of the subsystem.    
    /// \param name name of the value to display
    /// \param dtype indicates when the value should be displayed
    /// \param value the value to display
    public void putDashboard(String name, DisplayType dtype, double value) {
        if (shouldDisplay(dtype))
            SmartDashboard.putNumber(name, value) ;
    }

    /// \brief put a value on the driver station dashboard
    /// The dtype parameter controls when a value is displayed on the dashboard.  If dtype
    /// is set to DisplayType.Always, the value is always displayed on the dashboard.  If the
    /// dtype is set to Disabled, the value is only displayed when the robot is disabled.  This is
    /// useful for displaying values that aid in robot system checkes.  If the dtype is set to Verbose, 
    /// the value is only displayed when the robot is disabled or when the subsystem is in verbose 
    /// mode.  A subsystem is put in verbose mode, when the settings file contains a property with the
    /// name SUBSYSTEM:verbose set to true, where SUBSYSTEM is the name of the subsystem.    
    /// \param name name of the value to display
    /// \param dtype indicates when the value should be displayed
    /// \param value the value to display
    public void putDashboard(String name, DisplayType dtype, int value) {
        if (shouldDisplay(dtype))
            SmartDashboard.putNumber(name, value) ;
    }

    /// \brief put a value on the driver station dashboard
    /// The dtype parameter controls when a value is displayed on the dashboard.  If dtype
    /// is set to DisplayType.Always, the value is always displayed on the dashboard.  If the
    /// dtype is set to Disabled, the value is only displayed when the robot is disabled.  This is
    /// useful for displaying values that aid in robot system checkes.  If the dtype is set to Verbose, 
    /// the value is only displayed when the robot is disabled or when the subsystem is in verbose 
    /// mode.  A subsystem is put in verbose mode, when the settings file contains a property with the
    /// name SUBSYSTEM:verbose set to true, where SUBSYSTEM is the name of the subsystem.    
    /// \param name name of the value to display
    /// \param dtype indicates when the value should be displayed
    /// \param value the value to display    
    public void putDashboard(String name, DisplayType dtype, String value) {
        if (shouldDisplay(dtype))
            SmartDashboard.putString(name, value) ;        
    }

    /// \brief returns true if the subsystem is in verbose mode
    /// A subsystem is placed in verbose mode when there is a value in the settings file named
    /// SUBSYSTEM:verbose that is a boolean set to true.  This value is queried only when the subsystem
    /// is created and therefore changing the verbose setting on a running robot has no effect until
    /// the robot code is restarted.
    /// \returns true if the subsystem is in verbose mode
    public boolean isVerbose() {
        return verbose_ ;
    }

    /// \brief returns true if the subsystem is busy running and action and the action is not the default action
    /// \returns true if the subsystem is busy running an asction.    
    public boolean isBusy() {
        return action_ != null && !action_.isDone() && action_ != default_action_ ;
    }

    /// \brief returns true if the subsystem or any of its parents are busy
    /// returns true if the subsystem or any of its parents are busy
    public boolean isAnyParentBusy() {
        return isBusy() || (parent_ != null && parent_.isAnyParentBusy()) ;
    }

    /// \brief returns true if the subsystem or any of its children are busy
    /// \returns true if the subsystem or any of its children are busy
    public boolean isBusyOrChildBusy() {
        if (isBusy())
            return true ;

        for(Subsystem child : children_)
        {
            if (child.isBusyOrChildBusy())
                return true ;
        }

        return false ;
    }

    /// \brief initialize a plot with the name given.
    /// The name must be unique across all plots created.
    /// \param name the name of the plot
    /// \returns a handle to the plot to be used in any subsequent plot related calls.
    public int initPlot(String name) {
        return getRobot().getPlotManager().initPlot(name) ;
    }

    /// \brief start a plot with the data columns given
    /// \param id the handle for a plot returned by initPlot().
    /// \param cols an array of data columns to be plotted
    public void startPlot(int id, String[] cols) {
        getRobot().getPlotManager().startPlot(id, cols) ;
    }

    /// \brief add data to a plot
    /// \param id the handle for a plot returned by initPlot()
    /// \param data the data for the plot, should be the same size as the cols array in startPlot()
    public void addPlotData(int id, Double[] data) {
        getRobot().getPlotManager().addPlotData(id, data) ;
    }

    /// \brief end a plot
    /// This signals to any software listening to plots that this plot is complete and all data is present
    /// \param id the handle for a plot returned by initPlot()
    public void endPlot(int id) {
        getRobot().getPlotManager().endPlot(id) ;
    }

    /// \brief stub version of the computeMyState method.
    /// Should be implemented by a derived class
    protected void computeMyState() throws Exception {        
    }

    private boolean shouldDisplay(DisplayType dtype) {
        boolean ret = false ;

        switch(dtype) {
            case Always:
                ret = true ;
                break ;

            case Verbose:
                if (verbose_ || getRobot().isDisabled())
                    ret = true ;
                break ;

            case Disabled:
                if (getRobot().isDisabled())
                    ret = true ;
                break ;
        }

        return ret ;
    }

    private void cancelAction(boolean start_default) {
        if (action_ == null)
        {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            logger.add("Subsystem cancelAction() called with no active action").endMessage();
            return ;
        }

        action_.cancel() ;
        if (default_action_ != null && action_ != default_action_ && start_default)
        {
            action_ = default_action_ ;
            try {
                action_.start() ;
                if (action_.isDone())
                    finished_default_ = true ;
            }
            catch(Exception ex) {
                MessageLogger logger = getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("action ").addQuoted(action_.toString()) ;
                logger.add(" threw exception during start() - ").add(ex.getMessage()) ;
                logger.endMessage();
                getRobot().logStackTrace(ex.getStackTrace());
            }
        }
        else
        {
            action_ = null ;
        }
    }

    private void cancelActionPlusChildren() {
        if (action_ != null && !action_.isDone())
            cancelAction(false);

        for(Subsystem child: children_)
            child.cancelActionPlusChildren();
    }
}
