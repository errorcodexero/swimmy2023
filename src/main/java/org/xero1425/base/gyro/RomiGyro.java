// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.xero1425.base.gyro;

import org.xero1425.misc.XeroMath;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.hal.SimDouble;

/// \file

/// \brief This class represents the ROMI Gyro on the Romi robot
public class RomiGyro implements Gyro, XeroGyro {
  private SimDouble m_simRateX;
  private SimDouble m_simRateY;
  private SimDouble m_simRateZ;
  private SimDouble m_simAngleX;
  private SimDouble m_simAngleY;
  private SimDouble m_simAngleZ;

  private double m_angleXOffset;
  private double m_angleYOffset;
  private double m_angleZOffset;

  private SimDevice m_gyroSimDevice;

  /// \brief Create a new Romi gyro object
  public RomiGyro() {
    m_gyroSimDevice = SimDevice.create("Gyro:RomiGyro");
    if (m_gyroSimDevice != null) {
      m_gyroSimDevice.createBoolean("init", Direction.kOutput, true);
      m_simRateX = m_gyroSimDevice.createDouble("rate_x", Direction.kInput, 0.0);
      m_simRateY = m_gyroSimDevice.createDouble("rate_y", Direction.kInput, 0.0);
      m_simRateZ = m_gyroSimDevice.createDouble("rate_z", Direction.kInput, 0.0);

      m_simAngleX = m_gyroSimDevice.createDouble("angle_x", Direction.kInput, 0.0);
      m_simAngleY = m_gyroSimDevice.createDouble("angle_y", Direction.kInput, 0.0);
      m_simAngleZ = m_gyroSimDevice.createDouble("angle_z", Direction.kInput, 0.0);
    }
  }

  /// \brief Return the X rate of change
  /// \returns the X rate of change
  public double getRateX() {
    if (m_simRateX != null) {
      return m_simRateX.get();
    }

    return 0.0;
  }

  /// \brief Return the Y rate of change
  /// \returns the T rate of change
  public double getRateY() {
    if (m_simRateY != null) {
      return m_simRateY.get();
    }

    return 0.0;
  }

  /// \brief Return the Z rate of change
  /// \returns the Z rate of change
  public double getRateZ() {
    if (m_simRateZ != null) {
      return m_simRateZ.get();
    }

    return 0.0;
  }

  /// \brief Return the X angle
  /// \returns the X angle
  public double getAngleX() {
    if (m_simAngleX != null) {
      return m_simAngleX.get() - m_angleXOffset;
    }

    return 0.0;
  }

  /// \brief Return the Y angle
  /// \returns the Y angle
  public double getAngleY() {
    if (m_simAngleY != null) {
      return m_simAngleY.get() - m_angleYOffset;
    }

    return 0.0;
  }

  /// \brief Return the Z angle
  /// \returns the Z angle
  public double getAngleZ() {
    if (m_simAngleZ != null) {
      return m_simAngleZ.get() - m_angleZOffset;
    }

    return 0.0;
  }

  /// \brief Reset the gyro all to zero
  public void reset() {
    if (m_simAngleX != null) {
      m_angleXOffset = m_simAngleX.get();
      m_angleYOffset = m_simAngleY.get();
      m_angleZOffset = m_simAngleZ.get();
    }
  }

  /// \brief close the gyro connection
  @Override
  public void close() throws Exception {
    if (m_gyroSimDevice != null) {
      m_gyroSimDevice.close();
    }
  }

  /// \brief calibrate the gyro, does not do anything for the Romi.  Calbration is
  /// done via the Web Interface to the Romi robot.
  @Override
  public void calibrate() {
    // no-op
  }

  /// \brief return the total angle change from the last gyro reset
  @Override
  public double getAngle() {
    return -getAngleZ();
  }

  /// \brief return the rate of change of the gyro Z axis
  @Override
  public double getRate() {
    return getRateZ();
  }

  /// \brief return the effective YAW angle.
  @Override
  public double getYaw() {
    return XeroMath.normalizeAngleDegrees(-getAngleZ()) ;
  }

  /// \brief Return true if the Romi GYRO is connected
  @Override
  public boolean isConnected() {
    return m_gyroSimDevice != null ;
  }

  public double getGyroX() {
    return 0.0 ;
  }

  public double getGyroY() {
    return 0.0 ;
  }

  public double getGyroZ()  {
    return 0.0 ;
  }
  
  public double getAccelX()  {
    return 0.0 ;
  }

  public double getAccelY() {
    return 0.0 ;
  }

  public double getAccelZ() {
    return 0.0 ;
  }
}
