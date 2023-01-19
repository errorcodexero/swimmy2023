package org.xero1425.base.subsystems.tankdrive;

import org.xero1425.misc.XeroPath;

/// \file

/// \brief This class is the base class for all tank drive actions that follow a path
///
/// There are multiple path following approaches implemented in the Xero Framework.  This
/// class is the base class for each of them.
///
public abstract class TankDrivePathAction extends TankDriveAction {
    private String path_name_ ;
    private XeroPath path_ ;

    static private int logger_id_ = -1 ;
    static final private String PathFollowingLoggerID = "pathfollowing" ;

    /// \brief Create a new object
    /// \param sub the tank drive subsystem
    /// \param pathname the name of the path being followed
    public TankDrivePathAction(TankDriveSubsystem sub, String pathname) {
        super(sub) ;

        path_name_ = pathname ;
      
        if (logger_id_ == -1)
        {
            logger_id_ = getSubsystem().getRobot().getMessageLogger().registerSubsystem(PathFollowingLoggerID) ;
        }
    }

    /// \brief Return the logger ID for the action
    /// \returns the logger ID for the action
    public int getActionLoggerID() {
        return logger_id_ ;
    }

    /// \brief Return the path name for the action
    /// \returns the path name for the action
    public String getPathName() {
        return path_name_ ;
    }

    /// \brief Return the path for the action
    /// \returns the path for the action
    public XeroPath getPath() {
        return path_ ;
    }

    /// \brief called to start the action
    /// \exception throws MissingPathException if a path with the requested name does not exist
    public void start() throws Exception {
        super.start() ;
        path_ = getSubsystem().getRobot().getPathManager().getPath(getPathName()) ;
    }
}
