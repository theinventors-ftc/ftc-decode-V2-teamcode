package org.firstinspires.ftc.teamcode.Filters;

import java.util.function.DoubleSupplier;

public class IIRFilter {
    DoubleSupplier value;
    double smoothing_constant;
    double smoothed_value, raw_value;

    public IIRFilter(double smoothing_constant, DoubleSupplier value){
        this.value = value;
        this.smoothing_constant = smoothing_constant;
    }

    public void calculate() {
        smoothed_value = (1-smoothing_constant) * value.getAsDouble()
            + smoothing_constant * smoothed_value;
    }

    public void calculate(double given) {
        smoothed_value = (1-smoothing_constant) * given
            + smoothing_constant * smoothed_value;
    }

    public void set(double new_smoothing_constant){
        smoothing_constant = new_smoothing_constant;
    }

    public void reset(){
        smoothed_value = 0;
    }

    public double get() {
        calculate();
        return smoothed_value;
    }

    public double get(double given) {
        calculate(given);
        return smoothed_value;
    }

    public double get_raw() {
        return raw_value;
    }
}
