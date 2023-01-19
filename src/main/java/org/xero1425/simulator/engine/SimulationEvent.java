package org.xero1425.simulator.engine ;

public abstract class SimulationEvent {

    public SimulationEvent(double t) {
        time_ = t ;
    }

    public double getTime() {
        return time_ ;
    }

    public abstract String toString() ;
    public abstract void run(SimulationEngine enging) ;

    private double time_ ;
}
