package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.pedropathing.math.MathFunctions;

public class RPMAprilTagDistanceEquation {

    public static double DepoSpeed (double goalDist) {
        return MathFunctions.clamp(-0.00000554946 * Math.pow(goalDist, 4) + 0.00258915 * Math.pow(goalDist,3) - 0.41623 * Math.pow(goalDist, 2) + 28.18997 * (goalDist) + 122.72481, 800, 900);
    }

    public static double DepoSpeed1 (double goalDist){
        return  MathFunctions.clamp(-0.00000513336 * Math.pow(goalDist, 4)  + 0.00227959 * Math.pow(goalDist, 3) - 0.346971 * Math.pow(goalDist, 2) + 22.33346 * (goalDist) + 285.15456, 780, 920);
    }
    public static double DepoSpeed2 (double goalDist){
        return  MathFunctions.clamp(0.00000232388 * Math.pow(goalDist, 4)  - 0.000209333 * Math.pow(goalDist, 3) - 0.0850453 * Math.pow(goalDist, 2) + 13.94713 * (goalDist) + 261.57079, 780, 920);
    }

    public static double DepoSpeed3 (double goalDist){
        return  MathFunctions.clamp(0.00000194408 * Math.pow(goalDist, 4)  - 0.000667421 * Math.pow(goalDist, 3) + 0.0730403 * Math.pow(goalDist, 2) - 1.43476 * (goalDist) + 724.48454, 780, 920);
    }
    public static double DepoSpeed4 (double goalDist){
        return  MathFunctions.clamp(0.00000132982 * Math.pow(goalDist, 4)  - 0.000550131 * Math.pow(goalDist, 3) + 0.0781093 * Math.pow(goalDist, 2) -3.19447 * (goalDist) + 786.82072, 770, 920);
    }

    public static double DepoSpeed5 (double goalDist){
        return  MathFunctions.clamp(-0.0000018442 * Math.pow(goalDist, 4)  +0.000870702 * Math.pow(goalDist, 3) -0.156681 * Math.pow(goalDist, 2) +13.46395 * (goalDist) + 361.23254, 770, 920);
    }

    public static double DepoSpeed6 (double goalDist){
        return  MathFunctions.clamp(-0.00000459229 * Math.pow(goalDist, 4)  +0.0022458 * Math.pow(goalDist, 3) -0.411268 * Math.pow(goalDist, 2) +34.10407 * (goalDist) -256.11769, 770, 920);
    }
    public static double DepoSpeed7 (double goalDist){
        return  MathFunctions.clamp(- 0.00000569338 * Math.pow(goalDist, 4)  +0.00246149 * Math.pow(goalDist, 3) -0.375414 * Math.pow(goalDist, 2) +24.62591 * (goalDist) +179.82739, 750, 900);
    }
}



//y= -0.00000554946 x^{4} + 0.00258915 x^{3} -0.41623 x^{2} + 28.18997 x +122.72481

// y = -0.00000513336x^{4}+0.00227959x^{3}-0.346971x^{2}+22.33346x+285.15456

// y = 0.00000232388x^{4}-0.000209333x^{3}-0.0850453x^{2}+13.94713x+261.57079

// y = 0.00000194408x^{4}-0.000667421x^{3}+0.0730403x^{2}-1.43476x+724.48454

// y = 0.00000132982x^{4}-0.000550131x^{3}+0.0781093x^{2}-3.19447x+786.82072

// y = -0.0000018442x^{4}+0.000870702x^{3}-0.156681x^{2}+13.46395x+361.23254

// y = -0.00000459229x^{4}+0.0022458x^{3}-0.411268x^{2}+34.10407x-256.11769

// y= -0.00000569338x^{4}+0.00246149x^{3}-0.375414x^{2}+24.62591x+179.82739