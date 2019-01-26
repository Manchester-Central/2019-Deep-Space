/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class KYSPID {

    double P, I, D ;
    double error, previousError, errorSum;
    double setPoint;

    public KYSPID (double P, double I, double D, double setPoint) {

        reset(P, I, D, setPoint);

    }

    public void reset (double P, double I, double D, double setPoint) {
        
        this.P = P;
        this.I = I;
        this.D = D;
        this.setPoint = setPoint;
        error = 0D;
        previousError = 0D;
        errorSum = 0D;

    }

    public void setSetPoint (double newSetPoint) {

        reset(P, I, D, newSetPoint);
    }

    public void setPIDs (double P, double I, double D) {
        
        this.P = P;
        this.I = I;
        this.D = D;
    

    }

    public double getPIDSpeed (double currentPosition) {

        error = setPoint - currentPosition;
        errorSum += (error * 0.2);
        double changeInError = error - previousError;

        System.out.println("Proportional = " + P*error);
        return ((P * error) + (I * errorSum) + (D * changeInError));


    }

    public double getP () {return P;}
    public double getI () {return I;}
    public double getD () {return D;}
    public double getError () {return error;}
    public double getErrorSum () {return errorSum;}
    public double getSetPoint () {return setPoint;}
    public double getPreviousError () {return previousError;}
}
