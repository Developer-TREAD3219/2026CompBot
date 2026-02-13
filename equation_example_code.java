public class App {

    /**
     * Calculates the value based on the provided formula.
     *
     * @param thetaApril The angle θ_april in degrees.
     * @param distApril     The value for D_april.
     * @return The result of the calculation.
     */
    public static double calculateEquation(double thetaApril, double distApril) {
        // Define the constant value
        final double CONSTANT = 23.5;

        // --- Numerator Calculation ---
        // Numerator: 23.5 * (θ_april + 90°)
        double numerator = CONSTANT * (thetaApril + 90.0);


        // --- Denominator Calculation ---
        // First, convert the angle to radians for the cosine function
        double angleInRadians = Math.toRadians(thetaApril + 90.0);

        // Calculate the terms inside the square root
        // Term 1: 23.5²
        double term1 = CONSTANT * CONSTANT;
        // Term 2: D_april
        double term2 = distApril;
        // Term 3: 2 * 23.5 * D_april * cos(θ_april + 90°)
        double term3 = Math.abs(2.0 * CONSTANT * distApril * Math.cos(angleInRadians));

        // Denominator: √(term1 + term2 + term3)
        System.out.println("Term 1 (23.5²): " + term1);
        System.out.println("Term 2 (D_april): " + term2);
        System.out.println("Term 3 (2 * 23.5 * D_april * cos(θ_april + 90°)): " + term3);
        double denominator = Math.sqrt(term1 + term2 + term3);

        // --- Final Calculation ---
        // It's good practice to check for division by zero, though unlikely with this formula.
        if (denominator == 0) {
            throw new ArithmeticException("Denominator cannot be zero.");
        }
        System.out.println("Numerator: " + numerator);
        System.out.println("Denominator: " + denominator);
        return numerator / denominator;
    }

    /**
     * Main method for testing the function.
     */
    public static void main(String[] args) {
        // Example input values
        double theta = 30.0; // θ_april in degrees
        double distance = 100.0;    // D_april

        // Call the function
        double result = calculateEquation(theta, distance);

        // Print the result
        System.out.println("For θ_april = " + theta + "° and D_april = " + distance);
        System.out.println("Result: " + result);
    }
}
