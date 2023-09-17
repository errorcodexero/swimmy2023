package org.xero1425.base.gyro;

/// \file

/// \brief This interface defines the contract any Gyro must meet to be part of the
/// XeroFramework.
public interface XeroGyro {

    /// \brief Returns true if the Gyro is connected
    /// \returns true if the Gyro is connected, othewise false
    public boolean isConnected() ;

    /// \brief Returns the current effective YAW angle for the Gyro.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective YAW angle for the Gyro    
    public double getYaw() ;

    /// \brief Returns the current effective PITCH angle for the Gyro.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective PITCH angle for the Gyro     
    public double getPitch() ;

    /// \brief Returns the current effective ROLL angle for the Gyro.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective ROLL angle for the Gyro      
    public double getRoll();

    /// \brief Returns the total YAW angle for the Gyro.  This value will not wrap but will be cumlative.
    /// \returns the total YAW angle for the Gyro     
    public double getAngle() ;

    // public double getGyroX() ;
    // public double getGyroY() ;
    // public double getGyroZ() ;
    
    // public double getAccelX() ;
    // public double getAccelY() ;
    // public double getAccelZ() ;

    // public void reset() ;
}
