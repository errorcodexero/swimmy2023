package org.xero1425.base;

import java.io.File;
import java.io.IOException;
import java.net.NetworkInterface;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.JsonSettingsParser;
import org.xero1425.misc.MessageDestination;
import org.xero1425.misc.MessageDestinationFile;
import org.xero1425.misc.MessageDestinationThumbFile;
import org.xero1425.misc.SimArgs;
import org.xero1425.misc.XeroPathManager;
import org.xero1425.misc.XeroPathType;
import org.xero1425.base.motors.MotorFactory;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.Subsystem.DisplayType;
import org.xero1425.base.subsystems.tankdrive.TankDrivePathFollowerAction;
import org.xero1425.base.actions.Action;
import org.xero1425.base.controllers.BaseController;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.controllers.TeleopController;
import org.xero1425.base.controllers.TestController ;

/// \file

/// \brief This is the base class for any XeroFramework robot.  It is derived from the
/// WPILib TimedRobot class and provides a complete infrastructure for robot code including
/// a settings file, a message logger, and a plotting system.  It is expected that a class will
/// be derived from this class that is specific to the robot being programmed.
public abstract class XeroRobot extends TimedRobot {

    // This object contains important file system paths for the robot code
    private final RobotPaths robot_paths_;

    // The amount of time between the current robot loop and the previous robot loop.  Should be
    // approximately 20 ms but may vary some.
    private double delta_time_ ;

    // The time of the last robot loop
    private double last_time_;

    // The message logger for the robot
    private MessageLogger logger_ ;

    // The settings file supplier for the robot
    private ISettingsSupplier settings_ ;

    // The plot manager for the robot
    private IPlotManager plot_mgr_ ;

    // The path following paths
    private XeroPathManager paths_ ;

    // The motor factor for creating new motors
    private MotorFactory motors_ ;

    // The MAC address for the ethernet controller
    private byte[] mac_addr_ ;

    // The base robot subsystem
    private RobotSubsystem robot_subsystem_ ;

    // The current automode number as provided by the OI
    private int automode_ ;

    // The game data as provided by the WPILib Driver Station APIs
    private String game_data_ ;

    // if true, we have an FMS connection.  If true, the ploting manager is
    // disabled.
    private boolean fms_connection_ ;

    // The number of robot loops we have run in the current mode
    private int loop_count_ ;

    // The LOGGER id for the XeroRobot class
    private int logger_id_ ;

    // The current controller for the robot.  Will be one of the following three depending on the mode
    private BaseController current_controller_ ;

    // The auto controller for the robot
    private AutoController auto_controller_ ;

    // The teleop controller for the robot
    private TeleopController teleop_controller_ ;

    // The test controller for the robot
    private TestController test_controller_ ;

    // If not null, this is the compressor for pneumatics
    private Compressor compressor_ ;

    // The type of pneumatics on the robot
    private PneumaticsModuleType pneumatics_type_ ;

    private PowerDistribution pdp_ ;

    // Server for dispalying the status of the robot
    // private StatusServer server_ ;

    // The april tag layout for this field
    private AprilTagFieldLayout layout_ ;

    /// \brief The "subsystem" name for the message logger for this class
    public static final String LoggerName = "xerorobot" ;

    // A array to convert hex characters to integers
    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

    private static final String PDPPropertyName = "system:pdp:type" ;

    /// \brief Create a new XeroRobot robot
    /// \param period the robot loop timing (generally 20 ms)
    public XeroRobot(final double period) {
        super(period);

        double start ;

        // Generate the paths to the various important places (logfile directory, settings file, path follow paths directoryh, etc.)
        robot_paths_ = new RobotPaths(RobotBase.isSimulation(), getName());

        // Setup the mesasge logger to log messages
        start = getTime() ;
        enableMessageLogger();
        logger_id_ = logger_.registerSubsystem(LoggerName) ;
        logger_.startMessage(MessageType.Info).add("============================================================").endMessage();
        logger_.startMessage(MessageType.Info).add("robot code starting").endMessage();
        logger_.startMessage(MessageType.Info).add("enableMessageLogger time", getTime() - start).endMessage();
        logger_.startMessage(MessageType.Info).add("============================================================").endMessage();

        if (RobotBase.isSimulation()) {
            String str = SimArgs.InputFileName;
            if (str == null)
                str = getSimulationFileName() ;

            if (str == null) {
                System.out.println("The code is setup to simulate, but the derived robot class did not provide a stimulus file") ;
                System.out.println("Not initializing the Xero1425 Simulation engine - assuming Romi robot") ;
            }
            else {
                SimulationEngine.initializeSimulator(this, logger_);
                addRobotSimulationModels() ;
                SimulationEngine.getInstance().initAll(str) ;
            }
        }

        start = getTime() ;
        // Get the network MAC address, used to determine comp bot versus practice bot
        getMacAddress();
        logger_.startMessage(MessageType.Info).add("getMacAddress time", getTime() - start).endMessage(); ;

        // Read the parameters file
        start = getTime() ;
        readParamsFile();
        logger_.startMessage(MessageType.Info).add("readParamsFile time", getTime() - start).endMessage() ;

        // Enable messages in the message logger based on params file values
        start = getTime() ;
        enableMessagesFromSettingsFile() ;
        logger_.startMessage(MessageType.Info).add("enableMessagesFromSettingsFile time", getTime() - start).endMessage() ;

        // Read the paths files needed
        start = getTime() ;
        paths_ = new XeroPathManager(logger_, robot_paths_.pathsDirectory(), getPathType());
        try {
            loadPathsFile();
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("caught exception reading path files -").add(ex.getMessage()).endMessage();
        }
        logger_.startMessage(MessageType.Info).add("loadPathsFiles time", getTime() - start).endMessage() ;

        // Create the motor factor
        motors_ = new MotorFactory(logger_, settings_);

        // Create the plot manager
        int ver ;
        
        try {
            ver = settings_.get("system:plotting:version").getInteger() ;
        }
        catch(Exception ex) {
            ver = 3 ;
        }

        if (ver == 4) {
            plot_mgr_ = new PlotManagerNT4("/XeroPlot") ;
        }
        else {
            //
            // In all cases fall back to the proven plotting unless the version
            // is explictly 4
            //
            plot_mgr_ = new PlotManager("/XeroPlot");
        }

        // Store the initial time
        last_time_ = getTime();

        automode_ = -1;

        // try {
        //     server_ = new StatusServer(9001) ;
        //     server_.start() ;
        // }
        // catch(IOException ex) {

        // }
    }

    public RobotPaths getRobotFileSystemPaths() {
        return robot_paths_ ;
    }

    public List<AutoMode> getAllAutomodes() {
        final ArrayList<AutoMode> none = new ArrayList<AutoMode>() ;

        if (auto_controller_ == null) {
            return none ;
        }
        
        return auto_controller_.getAllAutomodes() ;
    }

    /// \brief returns true if pneumatics are enabled
    /// \return true if pneumatics are enabled
    public boolean arePneumaticsEnabled() {
        return compressor_ != null ;
    }

    /// \brief returns the type of pneumatics enabled
    /// \returns the type of pneumatics enabled
    public PneumaticsModuleType getPneumaticsType() {
        if (compressor_ == null) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("getPneumaticsType() called before enablePneumatics() was called") ;
            logger_.endMessage();
        }
        return pneumatics_type_ ;
    }

    /// \brief returns the compressor for pneumatics
    /// \returns the the compressor for pneumatics
    public Compressor getCompressor() {
        return compressor_ ;
    }

    /// \brief enable pneumatics for this robot using a REVPH and an analog sensor
    protected void enablePneumaticsAnalog() throws Exception {
        enablePneumatics() ;

        if (getPneumaticsType() != PneumaticsModuleType.REVPH) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("calling enablePneumaticsAnalog with a 'CTREPCM' is not allowed") ;
            logger_.endMessage();

            throw new Exception("Bad call to enablePneumaticsAnalog") ;
        }

        double minp = settings_.get("system:pneumatics:min-pressure").getDouble();
        double maxp = settings_.get("system:pneumatics:max-pressure").getDouble();

        compressor_.enableAnalog(minp, maxp);
    }

    /// \brief enable pneumatics for this robot
    /// \returns true if pneumatics were enabled sucessfully, otherwise false
    protected boolean enablePneumatics() {
        SettingsValue v ;
        PneumaticsModuleType type = PneumaticsModuleType.REVPH ;

        v = settings_.getOrNull("system:pneumatics:type") ;
        if (v == null) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("enablePneumatics() called but settings value 'system:pneumatics:type' was not found") ;
            logger_.endMessage();
            return false ;
        }

        if (!v.isString()) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("enablePneumatics() called but settings value 'system:pneumatics:type' was not a string") ;
            logger_.endMessage();
            return false ;
        }

        try {
            if (v.getString().equals("CTREPCM")) {
                type = PneumaticsModuleType.CTREPCM ;
            }
            else if (v.getString().equals("REVPH")) {
                type = PneumaticsModuleType.REVPH ;
            }
            else {
                logger_.startMessage(MessageType.Error) ;
                logger_.add("enablePneumatics() called but settings value 'system:pneumatics:type' must be either 'CTREPCM' or 'REVPH'") ;
                logger_.endMessage();
                return false ;
            }
        }
        catch(BadParameterTypeException e) {
            // Will never happen
        }

        pneumatics_type_ = type ;
        compressor_ = new Compressor(pneumatics_type_) ;

        return true ;
    }

    /// \brief Returns the message logger id for this class
    /// \returns the message logger id for this class
    public int getLoggerID() {
        return logger_id_ ;
    }

    /// \brief Returns the simulation stimulus file (JSON) name.  This method should be overridden by the
    /// derived robot specific class.
    /// \returns the simulation stimulus file (JSON) name
    protected String getSimulationFileName() {
        return null ;
    }

    /// \brief Returns the type of paths needed based on the path following
    /// algorithm implemented for this robot.  Expected to be overridden by the derived robot specific class.
    /// \returns tye type of paths needed
    protected XeroPathType getPathType() {
        return XeroPathType.TankPathFollowing ;
    }

    /// \brief Set the top level robot subsystem.  A robot is implemented as a hierarchy of subsystems.  The top
    /// level subsystem is expected to be derived from the RobotSubsystem class and is expected to contain the
    /// drivebase subsystem and the OI subsystem.  This method provides the manner to set this top level subsystem
    /// class.
    public void setRobotSubsystem(RobotSubsystem sub) {
        robot_subsystem_ = sub;
    }

    /// \brief Returns the number of robot loops that have been executed in the current mode
    /// \returns the number of robot loops that have been executed in the current mode
    public int getLoopCount() {
        return loop_count_ ;
    }

    /// \brief Initialize the robot
    @Override
    public void robotInit() {
        boolean v;
        double start ;

        logger_.startMessage(MessageType.Info).add("initializing robot") ;
        if (DriverStation.isFMSAttached())
            logger_.add(" - FMS connected") ;
        else
            logger_.add(" - no FMS") ;
        logger_.endMessage();

        if (settings_.isDefined(PDPPropertyName)) {
            try {
                String pdptype = settings_.get(PDPPropertyName).getString() ;
                if (pdptype.equals("rev")) {
                    pdp_ = new PowerDistribution(1, ModuleType.kRev);
                }
                else if (pdptype.equals("ctre")) {
                    pdp_ = new PowerDistribution(0, ModuleType.kCTRE);
                }
            }
            catch(Exception ex) {
                logger_.startMessage(MessageType.Error);
                logger_.add("error initialzing PDP - " + ex.getMessage());
                logger_.endMessage();
            }
        }

        /// Initialize the plotting subsystem
        start = getTime() ;
        try {
            v = settings_.get("system:plotting:enabled").getBoolean();
            if (v == true)
                plot_mgr_.enable(true);
            else
                plot_mgr_.enable(false);
        } catch (Exception ex) {
            //
            // Either the parameter is missing, or is not a boolean. In either
            // case we just turn off plotting
            plot_mgr_.enable(false);
        }
        logger_.startMessage(MessageType.Info).add("plotMgrInit time", getTime() - start).endMessage() ;

        //
        // initialize the basic hardware
        //
        try {
            // Create the robot hardware
            start = getTime() ;
            hardwareInit();
            logger_.startMessage(MessageType.Info).add("hardwareInit time", getTime() - start).endMessage() ;

            if (RobotBase.isSimulation() && SimulationEngine.getInstance() != null)
            {
                //
                // If we are simulating, create the simulation modules required
                //
                SimulationEngine.getInstance().createModels() ;
            }
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error);
            logger_.add("exception thrown in hardwareInit() - ").add(ex.getMessage()).add("\n").add(ex.getStackTrace().toString()) ;
            logger_.endMessage();

            logStackTrace(ex.getStackTrace());

            signalHardwareInitFailure() ;

            robot_subsystem_ = null;
        }

        if (robot_subsystem_ == null) {
            logger_.startMessage(MessageType.Error);
            logger_.add("the robot subsystem was not set in hardwareInit()");
            logger_.endMessage();

            return;
        }

        // Now that all subsystem are in place, compute the initial state of the robot
        delta_time_ = getPeriod();
        start = getTime() ;
        try {
            robot_subsystem_.computeState();
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error);
            logger_.add("Exception caught in computeState() - ").add(ex.getMessage());
            logger_.endMessage();
            ;
        }
        logger_.startMessage(MessageType.Info).add("computeInitalState time", getTime() - start).endMessage() ;

        // Now perform any initialization that might depend on the subsystem hierarchy
        // being in place or the initial state of the subsystems being ready.
        start = getTime() ;
        try {
            robot_subsystem_.postHWInit();
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error);
            logger_.add("Exception caught in postHWInit() - ").add(ex.toString());
            logger_.endMessage();

            logStackTrace(ex.getStackTrace());
        }
        logger_.startMessage(MessageType.Info).add("postHWInit time", getTime() - start).endMessage() ;

        // Create the auto mode controller
        try {
            auto_controller_ = createAutoController();
            if (auto_controller_ != null && isSimulation()) {
                checkPaths() ;
            }
        }
        catch(Exception ex) {
            logger_.startMessage(MessageType.Error);
            logger_.add("Exception caught creating automode controller - ").add(ex.getMessage());
            logger_.endMessage();

            logStackTrace(ex.getStackTrace());
        }

        // Create the teleop controller
        try {
            teleop_controller_ = createTeleopController();
        }
        catch(Exception ex) {
            logger_.startMessage(MessageType.Error);
            logger_.add("Exception caught creating teleop controller - ").add(ex.getMessage());
            logger_.endMessage();

            logStackTrace(ex.getStackTrace());
        }
    }

    public double getCurrent(int channel) {
        double ret = Double.POSITIVE_INFINITY ;

        if (pdp_ != null) {
            ret = pdp_.getCurrent(channel);
        }

        return ret;
    }

    /// \brief Called from the base class to indicate we are entering auto mode.
    @Override
    public void autonomousInit() {
        if (robot_subsystem_ == null)
            return;

        updateAutoMode();
        logAutoModeState();

        current_controller_ = auto_controller_;
        if (current_controller_ != null)
            current_controller_.init();

        robot_subsystem_.init(LoopType.Autonomous);

        loop_count_ = 0 ;
    }

    /// \brief Called from the base class each robot loop while in autonmous
    @Override
    public void autonomousPeriodic() {
        if (robot_subsystem_ == null)
            return;


        robotLoop(LoopType.Autonomous);

        loop_count_++ ;
    }

    /// \brief Called from the base class to indicate we are entering teleop mode.
    @Override
    public void teleopInit() {
        if (robot_subsystem_ == null)
            return;

        logger_.startMessage(MessageType.Info).add("Starting teleop mode").endMessage();
        if (robot_subsystem_.getDB() != null) {
            logger_.startMessage(MessageType.Info).add("Drivebase Pose", robot_subsystem_.getDB().getPose()).endMessage();
        }
        else {
            logger_.startMessage(MessageType.Info).add("Drivebase Pose", "NONE").endMessage();
        }

        current_controller_ = teleop_controller_;
        if (current_controller_ != null)
            current_controller_.init();

        robot_subsystem_.init(LoopType.Teleop);

        loop_count_ = 0 ;
    }

    /// \brief Called from the base class each robot loop while in teleop
    @Override
    public void teleopPeriodic() {
        if (robot_subsystem_ == null)
            return;

        robotLoop(LoopType.Teleop);

        loop_count_++ ;
    }

    /// \brief Called from the base class to indicate we are entering test mode
    @Override
    public void testInit() {
        if (robot_subsystem_ == null)
            return;

        logger_.startMessage(MessageType.Info).add("Starting test mode").endMessage();

        current_controller_ = test_controller_;
        if (current_controller_ != null)
            current_controller_.init();

        robot_subsystem_.init(LoopType.Test);

        loop_count_ = 0 ;
    }

    /// \brief Called from the base class each robot loop while in test mode
    @Override
    public void testPeriodic() {
        if (robot_subsystem_ == null)
            return;

        loop_count_++ ;
    }

    /// \brief Called from the base class to indicate we are a simulation
    @Override
    public void simulationInit() {
    }

    /// \brief Called from the base class each robot loop while in simulation
    @Override
    public void simulationPeriodic() {
    }

    /// \brief Called when the robot enters the disabled state
    @Override
    public void disabledInit() {
        if (robot_subsystem_ == null)
            return;

        current_controller_ = null;
        robot_subsystem_.reset();

        automode_ = -1;
        robot_subsystem_.init(LoopType.Disabled);

        loop_count_ = 0 ;
    }

    // public void publishSubsystemStatus(String subsystem, String status) {
    //     server_.setSubsystemStatus(subsystem, status);
    // }

    /// \brief Called from the base class each robot loop while in the disabled state
    @Override
    public void disabledPeriodic() {
        if (robot_subsystem_ == null)
            return;

        double initial_time = getTime();
        delta_time_ = initial_time - last_time_;

        updateAutoMode();

        // server_.setRobotStatus("RobotStatus");
        // robot_subsystem_.publishStatus() ;

        try {
            robot_subsystem_.computeState();
        } catch (Exception ex) {
            logger_.startMessage(MessageType.Error);
            logger_.add("exception caught in computeState() in disabledPeriodic loop -");
            logger_.add(ex.getMessage());
            logger_.endMessage();
        }
        if (isSimulation()) {
            SimulationEngine engine = SimulationEngine.getInstance() ;
            if (engine != null)
                engine.run(getTime()) ;
        }

        last_time_ = initial_time;
        loop_count_++ ;
    }

    /// \brief Called from the base class, must be overridden
    @Override
    public void robotPeriodic() {
    }

    /// \brief Returns the top level robot subsystem
    /// \returns the top level robot subsystem
    public RobotSubsystem getRobotSubsystem() {
        return robot_subsystem_;
    }

    /// \brief Returns the current robot time in seconds
    /// \returns the current robot time in seconds
    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    /// \brief Returns the time between the last robot loop and the current robot loop
    /// \returns the time between the last robot loop and the current robot loop
    public double getDeltaTime() {
        return delta_time_;
    }

    /// \brief Returns the mesasge logger
    /// \returns the message logger
    public MessageLogger getMessageLogger() {
        return logger_;
    }

    /// \brief Returns the settings supplier
    /// \returns the setting supplier
    public ISettingsSupplier getSettingsSupplier() {
        return settings_;
    }

    /// \brief Returns the path following path manager
    /// \returns the path following path manager
    public XeroPathManager getPathManager() {
        return paths_;
    }

    /// \brief Returns the motor factory
    /// \returns the motor factory
    public MotorFactory getMotorFactory() {
        return motors_;
    }

    /// \brief Returns the mesasge logger
    /// \returns the message logger
    public IPlotManager getPlotManager() {
        return plot_mgr_;
    }

    /// \brief Signals that the robot initialization failed
    protected void signalHardwareInitFailure() {
        String msg = "hardware initialization failed - check the log file for details" ;
        DriverStation.reportError(msg, false) ;
    }

    /// \brief enable specific messages, epxected to be overridden by the derived class
    protected void enableMessages() {
    }

    /// \brief add specific models to the simulation, expected to be overridden by the derived class
    protected void addRobotSimulationModels() {
    }

    /// \brief return the name of the robot, expected to be overridden by the derived class
    public String getName() {
        return "YouShouldOverrideMeInTheDerivedClass" ;
    }

    /// \brief initialize the robot hardware.  Expected to be overridden by the derived classa
    protected void hardwareInit() throws Exception {
        throw new Exception("override this in the derived class");
    }

    // \brief return the MAC address for the practice bot, expected to be overridden by the derived class
    /// \returns the MAC address for the practice bot
    protected byte[] getPracticeBotMacAddress() {
        return new byte[] { 127, 122, -90, -4, 52, 93 } ;
    }

    /// \brief returns true if the current robot is the practice bot.  This is done by comparing the MAC
    /// address of the ethernet port on the RoboRio to a specific MAC address provided by the method getParcticeBotMacAddress().
    /// \returns true if the current robot is the practice bot
    protected boolean isPracticeBot() {
        if (mac_addr_ == null)
            return false;

        byte[] addr = getPracticeBotMacAddress();
        if (addr == null)
            return false;
        boolean ret = true ;
        for(int i = 0 ; i  < 6 ; i++)
        {
            byte b1 = addr[i] ;
            byte b2 = mac_addr_[i] ;
            if (b1 != b2)
                ret = false ;
        }
        return ret ;
    }

    /// \brief Abtract method to create the automode controller, must be overridden by the derived class
    /// \returns the automode controller
    protected abstract AutoController createAutoController() throws MissingParameterException, BadParameterTypeException ;

    /// \brief Create and return the teleop controller
    /// \returns the teleop controller
    protected TeleopController createTeleopController() throws MissingParameterException, BadParameterTypeException {
        return new TeleopController(this, getName() + "-teleop") ;
    }

    /// \brief Returns the auto mode selection as provided by the OI
    /// \returns the auto mode selection
    protected int getAutoModeSelection() {
        return robot_subsystem_.getOI().getAutoModeSelector() ;
    }

    /// \brief load the paths file from the paths file directory
    protected void loadPathsFile() throws Exception {
        XeroPathManager mgr = getPathManager() ;

        try (Stream<Path> walk = Files.walk(Paths.get(mgr.getBaseDir()))) {
            List<String> result = walk.map(x -> x.toString()).filter(f -> f.endsWith("-main.csv")).collect(Collectors.toList());
            for(String name : result) {
                int index = name.lastIndexOf(File.separator) ;
                if (index != -1) {
                    name = name.substring(index + 1) ;
                    name = name.substring(0, name.length() - 9) ;
                    mgr.loadPath(name) ;
                }
            }
        }
        catch(IOException ex) {
        }
    }

    private void enableMessagesFromSettingsFile() {
        String path = "system:messages" ;
        ISettingsSupplier p = getSettingsSupplier() ;
        MessageLogger m = getMessageLogger() ;

        var keys = p.getAllKeys(path) ;
        if (keys != null) {
            for(String key : keys)
            {
                try {
                    String longkey = path + ":" + key ;
                    SettingsValue v = p.get(longkey) ;
                    if (v.isBoolean() && v.getBoolean())
                    {
                        m.enableSubsystem(key) ;
                    }
                }
                catch(Exception ex)
                {
                }
            }
        }
    }

    private void robotLoop(LoopType ltype) {
        double initial_time = getTime() ;
        delta_time_ = initial_time - last_time_ ;

        if (getCompressor() != null)
            robot_subsystem_.putDashboard("Pressure", DisplayType.Always, getCompressor().getPressure()) ;

        if (isSimulation() && delta_time_ < 0.005) {
            //
            // When we run the simulation engine, we pause the time for the robot while we evaluate the
            // models.  I have noticed that this causes the periodic functions to be called back to back with
            // less than a few hundred micro-seconds of robot time between the calls.  This conditional obviously
            // is only valid during simulations and prevents robot loops with very small delta time values from
            // being processed.
            //
            return ;
        }

        logger_.startMessage(MessageType.Debug, logger_id_) ;
        logger_.add("xerorobot: starting loop,") ;
        logger_.add("time", initial_time) ;
        logger_.add("delta", delta_time_) ;
        logger_.endMessage() ;

        if (isSimulation()) {
            SimulationEngine engine = SimulationEngine.getInstance() ;
            if (engine != null) {
                engine.run(delta_time_) ;
            }
        }

        try {
            robot_subsystem_.computeState();
        }
        catch(Exception ex) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("exception caught in computeState() in robot loop -") ;
            logger_.add(ex.getMessage()) ;
            logger_.endMessage();

            logStackTrace(ex.getStackTrace());
        }

        if (current_controller_ != null)
            current_controller_.run() ;

        try {
            robot_subsystem_.run();
        }
        catch(Exception ex) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add("exception caught in run() in robot loop -") ;
            logger_.add(ex.getMessage()) ;
            logger_.endMessage();

            logStackTrace(ex.getStackTrace());
        }

        last_time_ = initial_time ;
    }

    public void logStackTrace(StackTraceElement [] trace) {
        if (isSimulation()) {
            //
            // Make if we stack trace and write to log file, the
            // simulation should fail.
            //
            SimulationEngine.getInstance().addAssertError() ;
        }
        logger_.logStackTrace(trace) ;
    }

    public AprilTagFieldLayout getAprilTags() {
        if (layout_ == null) {
            String path = robot_paths_.deployDirectory() + "/AprilTags.json" ;
        
            try {
              layout_ = new AprilTagFieldLayout(path) ;
            }
            catch (IOException ex) {
                logger_.startMessage(MessageType.Error).add("cannot load april tag file '" + path + "' - " + ex.getMessage()).endMessage();
                layout_ = null ;
            }
        }

        return layout_ ;
    }

    private void logAutoModeState() {
        logger_.startMessage(MessageType.Info) ;
        logger_.add("Entering Autonomous Mode").endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    Automode Number: ").add(automode_).endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    Automode Name: ").add(auto_controller_.getAutoModeName()).endMessage();


        String str = "undefined" ;
        if (DriverStation.getAlliance() == Alliance.Red)
            str = "RED" ;
        else if (DriverStation.getAlliance() == Alliance.Blue)
            str = "BLUE" ;
        logger_.startMessage(MessageType.Info) ;
        logger_.add("    Alliance: ").add(str).endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    Location: ").add(DriverStation.getLocation()).endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    GameData: '").add(game_data_).add("'").endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    EventName: ").add(DriverStation.getEventName()).endMessage();

        str = "invalid" ;
        switch(DriverStation.getMatchType()) {
            case None:
                str = "NONE" ;
                break ;
            case Elimination:
                str = "ELIMINATION" ;
                break ;
            case Qualification:
                str = "QUALIFICATION" ;
                break ;
            case Practice:
                str = "PRACTICE" ;
                break ;
        }
        logger_.startMessage(MessageType.Info) ;
        logger_.add("    MatchType: ").add(str).endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    MatchNumber: ").add(DriverStation.getMatchNumber()).endMessage();

        logger_.startMessage(MessageType.Info) ;
        logger_.add("    MatchTime: ").add(DriverStation.getMatchTime()).endMessage();

        str = "NO" ;
        if (DriverStation.isFMSAttached())
            str = "YES" ;
            logger_.startMessage(MessageType.Info) ;
            logger_.add("    FMS Attached: ").add(str).endMessage();
    }

    private void displayAutoModeState() {
        SmartDashboard.putNumber("AutoModeNumber", automode_) ;
        SmartDashboard.putString("AutoModeName", auto_controller_.getAutoModeName()) ;
    }

    private void updateAutoMode() {
        if (auto_controller_ != null) {
            String msg = DriverStation.getGameSpecificMessage() ;

            int sel = -1 ;

            if (robot_subsystem_.getOI() != null) {
                sel = getAutoModeSelection() ;
                if (sel != automode_ || msg.equals(game_data_) || DriverStation.isFMSAttached() != fms_connection_ || auto_controller_.isTestMode())
                {
                    automode_ = sel ;
                    game_data_ = msg ;
                    fms_connection_ = DriverStation.isFMSAttached() ;
                    displayAutoModeState() ;
                }
            }

            try {
                auto_controller_.updateAutoMode(automode_, game_data_) ;
            }
            catch(Exception ex)
            {
                logger_.startMessage(MessageType.Error).add("Exception thrown in updateAutoMode - ").add(ex.getMessage()).endMessage();
                logStackTrace(ex.getStackTrace());
            }

            AutoMode mode = auto_controller_.getAutoMode() ;
            if (mode != null) {
                Pose2d pose = mode.getInitialPose();
                DriveBaseSubsystem db = robot_subsystem_.getDB() ;
                if (db != null) {
                    db.setPose(pose);
                }
            }
        }
    }

    private void enableMessageLogger() {
        String logfile = SimArgs.LogFileName ;
        MessageDestination dest ;

        logger_ = new MessageLogger();
        logger_.setTimeSource(new RobotTimeSource());

        if (logfile != null) {
            dest = new MessageDestinationFile(logfile) ;
        }
        else {
            dest = new MessageDestinationThumbFile(robot_paths_.logFileDirectory(), 250);
        }
        logger_.addDestination(dest);
        enableMessages();
    }

    private void readParamsFile() {
        JsonSettingsParser file = new JsonSettingsParser(logger_);

        String bot ;
        if (isPracticeBot())
            bot = "PRACTICE" ;
        else
            bot = "COMPETITION" ;

        file.addDefine(bot) ;
        logger_.startMessage(MessageType.Info).add("reading params for bot ").addQuoted(bot).endMessage() ;

        String filename = robot_paths_.deployDirectory() + getName() + ".jsonc" ;

        File f = new File(filename) ;
        if (!f.exists()) {
            filename = robot_paths_.deployDirectory() + getName() + ".json" ;
            f = new File(filename) ;
            if (!f.exists()) {
                //
                // There is no params file
                //
                logger_.startMessage(MessageType.Error).add("no settings file exists for this robot").endMessage();
                return ;
            }
        }

        if (!file.readFile(filename)) {
            logger_.startMessage(MessageType.Error).add("error reading parameters file").endMessage();
        }

        settings_ = file ;
    }

    private void getMacAddress() {
        Enumeration<NetworkInterface> netlist ;
        mac_addr_ = null ;

        try {
            netlist = NetworkInterface.getNetworkInterfaces() ;
            while (netlist.hasMoreElements())
            {
                NetworkInterface ni = netlist.nextElement() ;
                String name = ni.getName() ;
                if (name.equals("eth0")) {
                    mac_addr_ = ni.getHardwareAddress() ;
                    break ;
                }
            }
        }
        catch(Exception ex)
        {
            mac_addr_ = null ;
        }

        logger_.startMessage(MessageType.Info).add("Mac Address: ") ;
        if (mac_addr_ == null)
            logger_.add("NONE") ;
        else
        {
            for(int j = 0 ; j < mac_addr_.length ; j++)
            {
                int v = mac_addr_[j] & 0xFF;
                if (j != 0)
                    logger_.add(':') ;
                logger_.add(HEX_ARRAY[v >>> 4]) ;
                logger_.add(HEX_ARRAY[v & 0x0F]) ;
            }
        }
        logger_.endMessage();
    }

    private void checkPaths() {
        boolean valid = true ;
        List<Action> actions = new ArrayList<Action>() ;
        List<AutoMode> modes = auto_controller_.getAllAutomodes();
        for(AutoMode mode : modes) {
            logger_.startMessage(MessageType.Debug, logger_id_) ;
            logger_.add("processing automode ").addQuoted(mode.getName()).endMessage();
            actions.clear(); ;
            mode.getAllChildren(actions);
            for(Action act : actions) {
                if (act instanceof TankDrivePathFollowerAction) {
                    TankDrivePathFollowerAction pa = (TankDrivePathFollowerAction)act ;
                    logger_.startMessage(MessageType.Debug, logger_id_) ;
                    logger_.add("    processing path ").addQuoted(pa.getPathName()).endMessage();

                    if (!paths_.hasPath(pa.getPathName())) {
                        logger_.startMessage(MessageType.Error) ;
                        logger_.add("automode ").addQuoted(mode.getName()) ;
                        logger_.add(" requires path ").addQuoted(pa.getPathName()) ;
                        logger_.add(" which is missing from the paths directory") ;
                        logger_.endMessage();
                        valid = false ;
                    }
                }
            }
            logger_.startMessage(MessageType.Debug, logger_id_) ;
            logger_.add("  contained ").add(actions.size()).add(" actions total").endMessage();
        }

        if (!valid) {
            logger_.startMessage(MessageType.Fatal).add("some required paths are missing").endMessage();
        }
    }

}
