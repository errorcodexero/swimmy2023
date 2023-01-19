package org.xero1425.simulator.models;

import org.xero1425.misc.EncoderMapper;
import org.xero1425.misc.XeroMath;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretSubModel {
    private SimMotorController motor_ ;
    private double angle_ ;
    private double degs_per_power_per_sec_ ;
    private int encoder_input_ ;
    private EncoderMapper mapper_ ;

    public TurretSubModel(SimMotorController motor, double deg_per_power_per_sec, int encoder, double rmax, double rmin, double emax, double emin, double rc, double ec) {
        motor_ = motor ;
        angle_ = 0.0 ;
        degs_per_power_per_sec_ = deg_per_power_per_sec ;

        encoder_input_ = encoder ;

        mapper_ = new EncoderMapper(rmax, rmin, emax, emin) ;
        mapper_.calibrate(rc, ec);

    }

    public double getAngle() {
        return angle_ ;
    }

    public Rotation2d getAngleRot() {
        return Rotation2d.fromDegrees(angle_) ;
    }

    public void run(double dt) {
        angle_ = XeroMath.normalizeAngleDegrees(angle_ + degs_per_power_per_sec_ * motor_.getPower() * dt) ;
        double voltage = mapper_.toEncoder(angle_) ;
        AnalogInDataJNI.setVoltage(encoder_input_, voltage);
    }
}
