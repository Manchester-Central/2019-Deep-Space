/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import frc.ChaosSensors.CanSparkEncoder;
import frc.ChaosSensors.ChaosBetterCANSpark;
import frc.ChaosSensors.ChaosBetterTalonSRX;

/**
 * Add your docs here.
 */
public class PIDLinked {

    //KYSPID[] pids;
    
    PIDController[] pids;
    ChaosBetterCANSpark[] sparks;

    public final double TURN_GAINS = .1;

    public PIDController[] getPids () {return pids;}

    public PIDLinked (PIDController ... pids) {

        this.pids = pids;
        
    }

    public void setSparks (ChaosBetterCANSpark ... sparks) {

        this.sparks = sparks;

    }

    public void setPIDValues (double P, double I, double D, double F) {

        for (PIDController pid : pids) {

            pid.setPID(P, I, D, F);

        }

    }

    public void stop () {
        for (PIDController pid : pids) {
            pid.disable();
        }
    }

    /***
     * set setpoint for each pid
     * @param setPoints setpoits 
     */
    public void set (double ... setPoints) {

        int i = 0;
        for (double setPoint : setPoints) {

            pids[i].setSetpoint(setPoint);

            i++;

        }

    }

    public void drive () {

        int length = pids.length;

        double averageError = 0D;

        for (PIDController pid : pids) {
            
            averageError += pid.getError() / pid.getSetpoint();




        }

        averageError /= length;

       // System.out.print ("average error: " + averageError + ", ");

        for ( int i = 0; i < length; i++ ) {
            
            PIDController pid = pids[i];

            sparks[i].setAdjustment(((pid.getError() / pid.getSetpoint()) - averageError) * TURN_GAINS);

           // System.out.print ("adjustment " + i + ": " + ((pid.getError() / pid.getSetpoint()) - averageError) + ", ");

            pid.enable();

        }

    }

    public void enableSpecificPID (int index){

        pids[index].enable();

    }

}
