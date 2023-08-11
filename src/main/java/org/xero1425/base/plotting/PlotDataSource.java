package org.xero1425.base.plotting;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class PlotDataSource {
    class OneEntry
    {
        public String name ;
        public Supplier<Double> source ;

        public OneEntry(String n, Supplier<Double> s) {
            this.name = n ;
            this.source = s ;
        }
    }

    private List<OneEntry> data_sources_ ;

    public PlotDataSource() {
        data_sources_ = new ArrayList<OneEntry>() ;
    }

    public int size() {
        return data_sources_.size() ;
    }

    public String getColName(int index) {
        return data_sources_.get(index).name ;
    }

    public Supplier<Double> getSupplier(int index) {
        return data_sources_.get(index).source ;
    }

    public Double getData(int index) {
        return data_sources_.get(index).source.get();
    }

    public String[] columns() {
        String [] cols = new String[size()] ;
        for(int i = 0 ; i < size() ; i++) {
            cols[i] = getColName(i);
        }

        return cols ;
    }

    public Double[] values() {
        Double [] v = new Double[size()] ;
        for(int i = 0 ; i < size() ; i++) {
            v[i] = getData(i);
        }

        return v ;
    }

    public void addDataElement(String name, Supplier<Double> srcfn) {
        data_sources_.add(new OneEntry(name, srcfn));
    }

    
    public void convertUnits(String units) {
        for(int i = 0 ; i < data_sources_.size() ; i++) {
            String fixed = data_sources_.get(i).name.replace("%%units%%", units) ;
            data_sources_.get(i).name = fixed ;
        }
    }
}
