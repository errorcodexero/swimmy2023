package org.xero1425.base ;

import edu.wpi.first.wpilibj.Filesystem;

/// \file

/// \brief this class manages the paths for external files that the robot code must access
public class RobotPaths
{
    private String log_file_directory_ ;
    private String deploy_directory_ ;

    /// \brief create the robot paths object
    /// \param simulator if true we are running under the simulator
    /// \param name the name of the robot
    RobotPaths(boolean simulator, String name) {
        if (simulator)
        {
            log_file_directory_ = "logs" ;
            deploy_directory_ = "src/main/deploy/" ;
        }
        else
        {
            log_file_directory_ = "/u" ;
            deploy_directory_ = Filesystem.getDeployDirectory().getPath() + "/" ;
        }
    }

    /// \brief return the directory for log files
    /// \returns the directory for log files
    String logFileDirectory() {
        return log_file_directory_ ;
    }

    /// \brief return the directory for deployed files (e.g. the param file)
    /// \returns the directory for deployed files
    public String deployDirectory() {
        return deploy_directory_ ;
    }

    /// \brief return the directory that contains paths
    /// \returns the directory that contains files
    String pathsDirectory() {
        return deploy_directory_ + "/paths" ;
    }

} ;