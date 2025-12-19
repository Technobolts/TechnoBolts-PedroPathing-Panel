package TechnoBolts;
import java.lang.Math;
import static java.lang.Math.PI;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.atan;
import static java.lang.Math.toDegrees;

public class CalculateShooterAngle{

public static double calculateShooterAngle(double distance_x) {

        // --- 1. DEFINE YOUR ROBOT AND FIELD CONSTANTS HERE ---
        // !!! IMPORTANT: ENSURE ALL UNITS ARE CONSISTENT (ALL IN INCHES) !!!

        final double GRAVITY_CONST = --193.2;       // Use -4.905 for meters/s^2 OR -16.0 for feet/s^2 (-193.2 is for in/s^2)
        final double CONSTANT_RPM = 4500.0;         // Your fixed shooter wheel speed
        final double WHEEL_RADIUS = 5;           // Radius of your shooter wheel (e.g., in meters)
        final double ROBOT_HEIGHT = 10;           // Height of the ball's exit point (e.g., in meters)
        final double TARGET_HEIGHT = 50;           // Height of the center of the target goal (e.g., in meters)
        final double SHOOTER_EFFICIENCY = 0.4;      // TUNING CONSTANT: An efficeancy constant of how much of the motor's eneryg transferrs to the ball
        
        // --- 2. PHYSICS CALCULATIONS ---
        // 1. Calculate the theoretical speed of the wheel surface
        double V_theoretical = (2.0 * PI * WHEEL_RADIUS * CONSTANT_RPM) / 60.0;
        
        // 2. Apply the Efficiency/Slip Constant (Tuning Constant)
        // This accounts for energy lost when the ball compressed and friction.
        double V_actual = V_theoretical * SHOOTER_EFFICIENCY; 
        
        // 3. Square the ACTUAL velocity for use in the trajectory formula
        double V0_SQ = pow(V_actual, 2);
        
        // 4. Calculate the height difference
        double delta_h = TARGET_HEIGHT - ROBOT_HEIGHT;
        
        // 5. Calculate K using the adjusted velocity
        double K = (GRAVITY_CONST * pow(distance_x, 2)) / V0_SQ;
        
        // --- 3. QUADRATIC SOLUTION for tan(theta) ---
        // The form is: K*tan(theta)^2 + x*tan(theta) + (K - delta_h) = 0
        
        double A = K;
        double B = distance_x;
        double C = K - delta_h;

        // Calculate the Discriminant (D = B^2 - 4AC)
        double discriminant = pow(B, 2) - 4.0 * A * C;

        // Check for Unreachable Target
        if (discriminant < 0) {
            return Double.NaN; // Target is out of range
        }

        // Calculate tan(theta) for the LOWER angle solution
        // tan(theta)_low = (-B - sqrt(D)) / 2A (The minus sign ensures the lower angle is selected)
        double tan_theta_low = (-B - sqrt(discriminant)) / (2.0 * A);

        // Calculate the final angle and convert from radians to degrees
        double theta_low_rad = atan(tan_theta_low);
        double theta_low_deg = toDegrees(theta_low_rad);

        // --- 4. SERVO MAPPING ---

        // Define your physical mounting limits
        final double MIN_ANGLE_DEGREES = 20.0; //Need to be physically measured with a protractor
        final double MAX_ANGLE_DEGREES = 70.0; //Need to be physically measured with a protractor
        
        // Define your servo signal limits
        final double SERVO_MIN_SIGNAL = 0.4;
        final double SERVO_MAX_SIGNAL = 0.8;
        
        // Perform the map
        double servoPosition = SERVO_MIN_SIGNAL + (SERVO_MAX_SIGNAL - SERVO_MIN_SIGNAL) * (theta_low_deg - MIN_ANGLE_DEGREES) / (MAX_ANGLE_DEGREES - MIN_ANGLE_DEGREES);

        // THE SAFETY CLAMP
        // This ensures that even if the math says "Aim at 150 degrees," the servo stops at its physical limit.
        if (servoPosition < SERVO_MIN_SIGNAL) {
            servoPosition = SERVO_MIN_SIGNAL;
        } else if (servoPosition > SERVO_MAX_SIGNAL) {
            servoPosition = SERVO_MAX_SIGNAL;
        }
        
        // If the original math returned NaN (impossible shot), 
        // return a "home" or "safe" position.
        if (Double.isNaN(theta_low_deg)) {
            return SERVO_MIN_SIGNAL; 
        }
        
        return servoPosition; // Now returning the servo value instead of degrees
        
    }
}
