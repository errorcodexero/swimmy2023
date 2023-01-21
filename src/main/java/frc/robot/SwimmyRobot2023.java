// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SimArgs;
import org.xero1425.misc.XeroPathType;

import frc.robot.automodes.SwimmyRobotAutoController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class SwimmyRobot2023 extends XeroRobot {

  public SwimmyRobot2023() {
    super(0.02) ;
  }
  
  public String getName() {
    return "swimmy2023";
  }

  public String getSimulationFileName() {
      String ret = SimArgs.InputFileName;
      if (ret != null)
          return ret;

      return "automode";
  }


  protected void hardwareInit() throws Exception {
    
    enablePneumaticsAnalog() ;

    Swimmy2023RobotSubsystem robot = new Swimmy2023RobotSubsystem(this);
    setRobotSubsystem(robot);
  }

  @Override
  protected AutoController createAutoController() throws MissingParameterException, BadParameterTypeException {
    AutoController ctrl;
    try {
      ctrl = new SwimmyRobotAutoController(this);
    } catch (Exception ex) {
      ctrl = null;
    }

    return ctrl;
  }

  protected XeroPathType getPathType() {
    return XeroPathType.SwerveHolonomic ;
  }
}
