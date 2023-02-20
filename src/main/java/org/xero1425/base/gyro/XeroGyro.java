package org.xero1425.base.gyro;

/// \file

/// \brief This interface defines the contract any Gyro must meet to be part of the
/// XeroFramework.
public interface XeroGyro {

    /// \brief Returns true if the NavX is connected
    /// \returns true if the NavX is connected, othewise false
    public boolean isConnected() ;

    /// \brief Returns the current effective YAW angle for the NavX.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective YAW angle for the NavX    
    public double getYaw() ;
    public double getPitch() ;
    public double getRoll();

    /// \brief Returns the total angle for the NavX
    /// \returns the total angle for the NavX     
    public double getAngle() ;

    public double getGyroX() ;
    public double getGyroY() ;
    public double getGyroZ() ;
    
    public double getAccelX() ;
    public double getAccelY() ;
    public double getAccelZ() ;

    // public void reset() ;
}
