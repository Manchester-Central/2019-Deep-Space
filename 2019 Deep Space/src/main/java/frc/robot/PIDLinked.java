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
public class PIDLinked {

    KYSPID[] pids;
    Double[][] pidValues;

    public PIDLinked (KYSPID ... pids) {

        this.pids = pids;

        int i = 0;
        for (KYSPID pid : this.pids) {

            pidValues[i][0] = pid.getP();
            pidValues[i][1] = pid.getI();
            pidValues[i][2] = pid.getD();
            i ++;
        }

    }

    public void set (double ... setPoints) {

        int i = 0;
        for (double setPoint : setPoints) {

            Double[] pidV =  pidValues[i];

            pids[i].reset(pidV[0], pidV[1], pidV[2], setPoint);

            i++;

        }

    }

    public double[] getLinkedPID (double ... currentPositions) {

        double[] returnValues = new double[pids.length];

        double averageError = 0D;

        for (KYSPID pid : pids) {
            
            averageError += pid.getError() / pid.getSetPoint();

        }

        averageError /= pids.length;

        int i = 0;
        for (double returnValue : returnValues) {
            
            double adjustment = pids[i].getError() - averageError;

            returnValues[i] = pids[i].getPIDSpeed(currentPositions[i]) + adjustment;

            i++;
        }

        return returnValues;

    }

}
