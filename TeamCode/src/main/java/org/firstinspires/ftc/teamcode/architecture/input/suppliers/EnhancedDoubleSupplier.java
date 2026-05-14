package org.firstinspires.ftc.teamcode.architecture.input.suppliers;

import java.util.function.DoubleSupplier;

public class EnhancedDoubleSupplier {
    private final DoubleSupplier doubleSupplier;
    private boolean valid = false;
    private double current;

    public EnhancedDoubleSupplier(DoubleSupplier doubleSupplier) {
        this.doubleSupplier = doubleSupplier;
        this.current = doubleSupplier.getAsDouble();
    }

    public void invalidate() {
        valid = false;
    }

    public void primeToCurrentState() {
        current = doubleSupplier.getAsDouble();
        valid = true;
    }

    public double getState() {
        if (!valid) {
            current = doubleSupplier.getAsDouble();
            valid = true;
        }
        return current;
    }

    public ConditionalBinder conditionalBindState() {
        return new ConditionalBinder(this);
    }

    public EnhancedBooleanSupplier greaterThan(double threshold) {
        return new EnhancedBooleanSupplier(() -> this.getState() > threshold);
    }

    public EnhancedBooleanSupplier lessThan(double threshold) {
        return new EnhancedBooleanSupplier(() -> this.getState() < threshold);
    }

    public EnhancedBooleanSupplier greaterThanOrEqual(double threshold) {
        return new EnhancedBooleanSupplier(() -> this.getState() >= threshold);
    }

    public EnhancedBooleanSupplier lessThanOrEqual(double threshold) {
        return new EnhancedBooleanSupplier(() -> this.getState() <= threshold);
    }

    public EnhancedBooleanSupplier inRange(double min, double max) {
        return new EnhancedBooleanSupplier(() -> {
            double value = this.getState();
            return value >= min && value <= max;
        });
    }

    public static class ConditionalBinder {
        private final EnhancedDoubleSupplier supplier;
        private double minValue = Double.NEGATIVE_INFINITY;
        private double maxValue = Double.POSITIVE_INFINITY;
        private boolean minInclusive = false;
        private boolean maxInclusive = false;

        public ConditionalBinder(EnhancedDoubleSupplier supplier) {
            this.supplier = supplier;
        }

        public ConditionalBinder greaterThan(double value) {
            this.minValue = value;
            this.minInclusive = false;
            return this;
        }

        public ConditionalBinder greaterThanEqualTo(double value) {
            this.minValue = value;
            this.minInclusive = true;
            return this;
        }

        public ConditionalBinder lessThan(double value) {
            this.maxValue = value;
            this.maxInclusive = false;
            return this;
        }

        public ConditionalBinder lessThanEqualTo(double value) {
            this.maxValue = value;
            this.maxInclusive = true;
            return this;
        }

        public EnhancedBooleanSupplier bind() {
            return new EnhancedBooleanSupplier(() -> {
                double value = supplier.getState();
                boolean minCheck = minInclusive ? value >= minValue : value > minValue;
                boolean maxCheck = maxInclusive ? value <= maxValue : value < maxValue;
                return minCheck && maxCheck;
            });
        }
    }
}
