package org.xero1425.base.gyro;

import org.xero1425.base.XeroRobot;
import com.kauailabs.navx.frc.AHRS ;
import edu.wpi.first.wpilibj.SPI;

/// \file

/// \brief This class implements the XeroGyro interface for the NavX gyro
public class NavxGyro implements XeroGyro {

    // The instance of the NavX Gyro
    private AHRS navx_ ;

    /// \brief Create the Gyro class using the default NavX Port on the MXP bus
    public NavxGyro() {
        if (XeroRobot.isSimulation()) {
            navx_ = null ;
        }
        else {
            navx_ = new AHRS(SPI.Port.kMXP) ;
        }
    }

    /// \brief Create the Gyro class using the supplied SPI port
    /// \param port the SPI port to use to talk to the NavX hardware
    public NavxGyro(SPI.Port port) {
        if (XeroRobot.isSimulation()) {
            navx_ = null ;
        }
        else {
            navx_ = new AHRS(port) ;
        }
    }

    /// \brief Returns true if the NavX is connected
    /// \returns true if the NavX is connected, othewise false
    public boolean isConnected() {
        if (navx_ == null) {
            return true ;
        }

        return navx_.isConnected() ;
    }

    /// \brief Returns the current effective YAW angle for the NavX.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective YAW angle for the NavX
    public double getYaw() {
        double ret = 0.0 ;

        if (navx_ != null) {
            ret = -navx_.getYaw() ;
        }
        return ret ;
    }

    public double getPitch() {
        double ret = 0.0 ;

        if (navx_ != null) {
            ret = navx_.getPitch();
        }
        return ret ;
    }

    public double getRoll() {
        double ret = 0.0 ;

        if (navx_ != null) {
            ret = navx_.getRoll();
        }
        return ret ;
    }

    /// \brief Returns the total angle for the NavX
    /// \returns the total angle for the NavX    
    public double getAngle() {
        if (navx_ == null) {
            return 0.0 ;
        }

        return navx_.getAngle() ;
    }

    public double getGyroX() {
        if (navx_ == null) {
            return 0.0 ;
        }

        return navx_.getRawGyroX() ;
    }
    public double getGyroY() {
        if (navx_ == null) {
            return 0.0 ;
        }

        return navx_.getRawGyroY() ;
    }

    public double getGyroZ() {
        if (navx_ == null) {
            return 0.0 ;
        }

        return navx_.getRawGyroZ() ;
    }
    
    public double getAccelX() {
        if (navx_ == null) {
            return 0.0 ;
        }

        return navx_.getRawAccelX() ;
    }

    public double getAccelY() {
        if (navx_ == null) {
            return 0.0 ;
        }
        
        return navx_.getRawAccelY() ;
    }

    public double getAccelZ() {
        if (navx_ == null) {
            return 0.0 ;
        }
        
        return navx_.getRawAccelZ() ;
    }
}
