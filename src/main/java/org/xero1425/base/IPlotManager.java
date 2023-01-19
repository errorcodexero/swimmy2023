package org.xero1425.base;

public interface IPlotManager {
    public void enable(boolean b) ;
    public int initPlot(String name) ;
    public void startPlot(int id, String[] cols) ;
    public void addPlotData(int id, Double[] data) ;
    public void endPlot(int id) ;
}
