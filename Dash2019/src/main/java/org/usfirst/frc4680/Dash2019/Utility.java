/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4680.Dash2019;

/**
 * Add your docs here.
 */
public class Utility {
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
      }
    
    public static double squaredInput(double x) {
      x = clamp(x, -1.0, 1.0);
      return Math.signum(x) * x * x;
    }
}
