package org.xero1425.misc ;

import java.lang.Math ;

/// \file


/// \brief a set of general math functions
public class XeroMath
{
    /// \brief normalize an angle in degrees
    /// This method converts the input angle to an angle between -180 and +180 degrees
    /// \param a the angle in degrees
    /// \returns the input angle normalized to between -180 and +180 degrees
    public static double normalizeAngleDegrees(double a) {
        while (a <= -180.0)
            a += 360.0 ;

        while (a > 180.0)
            a -= 360.0 ;
        
        return a ;          
    }

    /// \brief normalize an angle in radians
    /// This method converts the input angle to an angle between -PI and +PI radians
    /// \param a the angle in radians
    /// \returns the input angle normalized to between -PI and +PI radians
    public static double normalizeAngleRadians(double a) {
        if (a <= -Math.PI)
            a += 2 * Math.PI   ;
        else if (a >Math.PI)
            a -= 2 *Math.PI   ;
        return a ;
    }

    /// \brief returns true if two double values are equal within a given percentage
    /// Note, care must be taken if the target approaches zero as percentange does not make sense
    /// when the target approaches zero.
    /// \param target the target value
    /// \param measured the measured value
    /// \param percentage the range by which the two values must match
    /// \returns true if the target and measured are within the percentage given
    public static boolean equalWithinPercentMargin(double target, double measured, double percentage) {
        return (Math.abs(100.0 * (measured - target)/target) <= percentage);
    }

    /// \brief returns the real roots of the quadratic given by a, b, and c
    /// The quadratic equation is given by a * x ^ 2 + b * x + c.  This function
    /// returns the real roots with the largest root always being the first value
    /// in the returned array.  If the two roots are repeated, only one root is given.
    /// No imaginary roots are returned.
    /// \param a the A coefficient of the quadratic question
    /// \param b the B coefficient of the quadratic question
    /// \param c the C coefficient of the quadratic question
    /// \returns a Double array with 0, 1, or 2 values which are the real roots of the quadratic
    public static double[] quadratic(double a, double b, double c) {
        double[] result ;
        double tmp = b * b - 4 * a * c ;

        if (tmp == 0.0) {
            result = new double[1] ;
            result[0] = -b/(2 * a) ;
        }
        else if (tmp > 0.0) {
            result = new double[2] ;
            result[0] = (-b + Math.sqrt(tmp)) / (2 * a) ;
            result[1] = (-b - Math.sqrt(tmp)) / (2 * a) ;

            if (result[0] < result[1]) {
                //
                // Swap the result, the biggest should always be first
                //
                tmp = result[0] ;
                result[0] = result[1] ;
                result[1] = tmp ;
            }
        }
        else {
            result = new double[0] ;
        }
        return result ;        
    }

    /// \brief returns the smallest real positive root for a quadratic equation with the roots given
    /// \param roots the set of roots of the quadratic
    /// \returns the single root that is the smallest positive root.  If none are present, returns MAX_VALUE.
    public static double pickRoot(double [] roots) throws Exception {
        if (roots.length == 0) {
            throw new Exception("no real roots for equation") ;
        }

        double minpos = Double.MAX_VALUE ;
        boolean found = false ;
        for(int i = 0 ; i < roots.length ; i++) {
            if (roots[i] >= 0.0 && roots[i] < minpos) {
                minpos = roots[i] ;
                found = true ;
            }
        }

        if (!found) {
            throw new Exception("no positive roots found for the equation") ;
        }

        return minpos ;
    }
}
