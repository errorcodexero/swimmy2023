package org.xero1425.base ;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import java.util.Map ;
import java.util.HashMap ;

/// \file

/// \brief This class manages "plots". 
///
/// Plots are data sets that are stored to the network table.  The data stored there
/// contains a set of points, per robot loop, for each robot loop that is processed while
/// the plot is ednabled.  A plot is defined by a name and a set of named columns.  Each named
/// column has a value for each robot loop.
/// This data can be processed by the xerotune toon located here <a href="https://www.mewserver.org/xeroprogs/" here </a>
public class PlotManagerNT4 implements IPlotManager
{
    private class PlotTable
    {
        public final String name_ ;
        public int columns_ ;
        public BooleanTopic completeTopic_ ;
        public BooleanPublisher completePublisher_ ;
        public StringArrayTopic columnNamesTopic_ ;
        public StringArrayPublisher columnNamesPublisher_ ;
        public IntegerTopic countTopic_ ;
        public IntegerPublisher countPublisher_ ;
        public IntegerTopic versionTopic_ ;
        public IntegerPublisher versionPublisher_ ;
        public DoubleTopic [] dataTopics_ ;
        public DoublePublisher [] dataPublishers_ ;
        public int count_ ;

        public PlotTable(int id, String name) {
            name_ = name ;
            columns_ = -1 ;
            completeTopic_ = null ;
            completePublisher_ = null ;
            columnNamesTopic_ = null ;
            columnNamesPublisher_ = null ;
            dataTopics_ = null ;
            dataPublishers_ = null ;
            count_ = 0 ;
        }
    } ;

    static private final String CompleteEntry = "complete" ;
    static private final String ColumnsEntry = "columns" ;
    static private final String DataEntry = "data" ;
    static private final String CountEntry = "count" ;
    static private final String VersionEntry = "version" ;
    
    private int next_plot_id_ ;
    private String plot_table_ ;
    private Map<Integer, PlotTable> plots_ ;
    private boolean enabled_ ;
    private XeroRobot robot_ ;

    /// \brief create a new plot manager
    /// \param key the name of the key in the network table to hold plot data
    public PlotManagerNT4(XeroRobot robot, String key)
    {
        plots_ = new HashMap<Integer, PlotTable>() ;
        next_plot_id_ = 0 ;
        plot_table_ = key ;
        enabled_ = false ;
        robot_ = robot ;
    }

    /// \brief enable or disable the storage of plotting data in the network tables
    /// \param b if true, enable plotting, otherwise disable
    public void enable(boolean b) {
        enabled_ = b ;
    }

    public int initPlot(String name)
    {
        if (!enabled_ || robot_.shutdownDebug())
            return -1 ;

        for(int key : plots_.keySet())
        {
            PlotTable table = plots_.get(key) ;
            if (table.name_.equals(name)) {
                return key ;
            }
        }

        int id = next_plot_id_++ ;
        PlotTable p = new PlotTable(id, name) ;
        plots_.put(id, p) ;

        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        String plotkey = getKeyForPlot(id) ;

        p.columnNamesTopic_ = inst.getStringArrayTopic(plotkey + "/" + ColumnsEntry) ;
        p.completeTopic_ = inst.getBooleanTopic(plotkey + "/" + CompleteEntry) ;
        p.countTopic_ = inst.getIntegerTopic(plotkey + "/" + CountEntry) ;
        p.versionTopic_ = inst.getIntegerTopic(plotkey + "/" + VersionEntry) ;

        return id ;
    }

    public void startPlot(int id, String[] cols)
    {
        if (!enabled_ || robot_.shutdownDebug() || !plots_.containsKey(id))
            return ;
      
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        PlotTable p = plots_.get(id) ;
        String plotkey = getKeyForPlot(id) ;

        p.columns_ = cols.length ;

        p.columnNamesPublisher_ = p.columnNamesTopic_.publish() ;
        p.columnNamesPublisher_.set(cols) ;

        p.completePublisher_ = p.completeTopic_.publish() ;
        p.completePublisher_.set(false) ;

        p.countPublisher_ = p.countTopic_.publish() ;
        p.countPublisher_.set(0) ;

        p.versionPublisher_ = p.versionTopic_.publish() ;
        p.versionPublisher_.set(4) ;

        p.dataTopics_ = new DoubleTopic[cols.length] ;
        p.dataPublishers_ = new DoublePublisher[cols.length] ;
        for(int i = 0 ; i < cols.length ; i++) {
            String dataname = plotkey + "/" + DataEntry + "/" + Integer.toString(i) ;
            p.dataTopics_[i] = inst.getDoubleTopic(dataname) ;
            p.dataPublishers_[i] = p.dataTopics_[i].publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true), PubSubOption.periodic(0.02)) ;
        }

        p.count_ = 0 ;
        
        inst.flush() ;
    }

    public void addPlotData(int id, Double[] data)
    {
        if (!enabled_ || robot_.shutdownDebug() || !plots_.containsKey(id))
            return ;

        PlotTable p = plots_.get(id) ;

        if (data.length != p.columns_)
            return ;

        for(int i = 0 ; i < p.columns_ ; i++) {
            p.dataPublishers_[i].set(data[i]) ;
        }

        p.count_++ ;
        p.countPublisher_.set(p.count_) ;

        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        inst.flush();        

        System.out.println("PlotManager: addPlotData: count " + p.count_) ;
    }

    public void endPlot(int id)
    {
        if (!enabled_ || robot_.shutdownDebug() || !plots_.containsKey(id))
            return ;

        PlotTable p = plots_.get(id) ;
        p.completePublisher_.set(true) ;
    }

    private String getKeyForPlot(int id)
    {
        if (!plots_.containsKey(id))
            return null ;

        PlotTable plot = plots_.get(id) ;
        return plot_table_ + "/" + plot.name_ ;
    }
} ;