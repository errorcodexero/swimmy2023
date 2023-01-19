package org.xero1425.misc;

public class MinMaxData {
    private double [] data_ ;
    private int index_ ;
    private int count_ ;

    public MinMaxData(int count) {
        data_ = new double[count] ;
        index_ = 0 ;
        count_ = 0 ;
    }

    public void addData(double d) {
        data_[index_++] = d ;
        if (count_ < data_.length) {
            count_++ ;
        }

        if (index_ == data_.length) {
            index_ = 0 ;
        }
    }

    public double getMin() {
        double ret = Double.MAX_VALUE ;
        for(int i = 0 ; i < count_ ; i++) {
            if (data_[i] < ret) {
                ret = data_[i] ;
            }
        }

        return ret ;
    }

    public double getMax() {
        double ret = Double.MIN_VALUE ;
        for(int i = 0 ; i < count_ ; i++) {
            if (data_[i] > ret) {
                ret = data_[i] ;
            }
        }

        return ret ;
    }

}
