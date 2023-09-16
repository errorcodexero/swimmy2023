package org.xero1425.base.gyro;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon2Gyro implements XeroGyro {
    private Pigeon2 gyro_ ;
    private double[] data_ ;

    public Pigeon2Gyro(int canid) {
        gyro_ = new Pigeon2(canid) ;
        data_ = new double[3] ;
    }

    public Pigeon2Gyro(int canid, String bus) {
        gyro_ = new Pigeon2(canid, bus) ;
    }

    public double getYaw() {
        return gyro_.getYaw() ;
    }

    public double getPitch() {
        return gyro_.getPitch() ;
    }

    public double getRoll() {
        return gyro_.getRoll() ;
    }

    /// \brief Returns the total angle for the NavX
    /// \returns the total angle for the NavX     
    public double getAngle()  {
        gyro_.getAccumGyro(data_) ;
        return data_[2] ;
    }

    public double getGyroX()  {
        gyro_.getRawGyro(data_) ;
        return data_[0] ;
    }

    public double getGyroY() {
        gyro_.getRawGyro(data_) ;
        return data_[1] ;
    }
    
    public double getGyroZ() {
        gyro_.getRawGyro(data_) ;
        return data_[2] ;        
    }
    
    public double getAccelX() ;
    public double getAccelY() ;
    public double getAccelZ() ;
}
