package frc.robot.mathUtil;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class LinearInterpolationTable {

    private final double[] xValues;
    private final double[] yValues;

    /**
     * Constructor
     * @param table 2xN array where:
     *              table[0] = x values (sorted ascending)
     *              table[1] = y values
     */
    public LinearInterpolationTable(double[][] table) {
        if (table == null || table.length != 2) {
            throw new IllegalArgumentException("Table must be a 2xN array.");
        }

        if (table[0].length != table[1].length) {
            throw new IllegalArgumentException("X and Y arrays must be same length.");
        }

        if (table[0].length < 2) {
            throw new IllegalArgumentException("Table must contain at least 2 points.");
        }

        this.xValues = table[0];
        this.yValues = table[1];

        validateSorted();
    }

    private void validateSorted() {
        for (int i = 1; i < xValues.length; i++) {
            if (xValues[i] <= xValues[i - 1]) {
                throw new IllegalArgumentException("X values must be strictly increasing.");
            }
        }
    }

    /**
     * Performs linear interpolation lookup.
     * @param x input value
     * @return interpolated y value
     */
    public double interpolate(double x) {

        // Clamp below minimum
        if (x <= xValues[0]) {
            return yValues[0];
        }

        // Clamp above maximum
        if (x >= xValues[xValues.length - 1]) {
            return yValues[yValues.length - 1];
        }

        // Find interval
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {

                double x0 = xValues[i];
                double x1 = xValues[i + 1];
                double y0 = yValues[i];
                double y1 = yValues[i + 1];

                // Linear interpolation formula
                return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
            }
        }

        throw new IllegalStateException("Interpolation failed.");
    }
}