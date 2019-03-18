/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.time.Clock;

/**
 * Add your docs here.
 */
public class RampComponent {
    // the maximum allowed change in the value per second 
    private final double maxChangePerMillis; 

    // value most recently returned
    private double lastValue; 

    // the tme, in milliseconsd, that the value most recently returned was returned at
    private long lastTime; 

    public RampComponent(double maxChangePerSecond){
        this.maxChangePerMillis = maxChangePerSecond / 1000.; 
    }

    // Ramp the given value: 
    
    public double applyAsDouble(double value){
        if(value > lastValue){
            lastValue = Math.min(value, lastValue + (System.currentTimeMillis() - lastTime) * maxChangePerMillis); 
        } else {
            lastValue = Math.max(value, lastValue - (System.currentTimeMillis() - lastTime) * maxChangePerMillis);
        }
        lastTime = System.currentTimeMillis(); 
        return lastValue; 

    }
    

}
