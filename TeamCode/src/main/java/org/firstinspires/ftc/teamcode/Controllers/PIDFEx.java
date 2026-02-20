package org.firstinspires.ftc.teamcode.Controllers;

import org.firstinspires.ftc.teamcode.Filters.IIRFilter;

public class PIDFEx {

        private IIRFilter filter;
        private double integralWorkingBounds = Double.POSITIVE_INFINITY,
                integralClippingBounds = Double.POSITIVE_INFINITY;

        private double kP, kI, kD, kF;
        private double alpha;
        private double setPoint;
        private double measuredValue;
        private double minIntegral, maxIntegral;

        private double errorVal_p;
        private double errorVal_p_filtered;
        private double errorVal_v;

        private double totalError;
        private double prevErrorVal_filtered;

        private double errorTolerance_p = 0.05;
        private double errorTolerance_v = Double.POSITIVE_INFINITY;

        private double lastTimeStamp;
        private double period;

        private double deadzone;

        public PIDFEx (
                double kp,
                double ki,
                double kd,
                double kf,
                double alpha,
                double deadzone,
                double integralWorkingBounds,
                double integralClippingBounds
        ) {
            this(
                    kp,
                    ki,
                    kd,
                    kf,
                    0,
                    0,
                    alpha,
                    deadzone,
                    integralWorkingBounds,
                    integralClippingBounds
            );
        }

        public PIDFEx(PIDFExCoeffs cons) {
            this(
                    cons.getkP(),
                    cons.getkI(),
                    cons.getkD(),
                    cons.getkF(),
                    cons.getAlpha(),
                    cons.getDeadzone(),
                    cons.getIntegralWorkingBounds(),
                    cons.getIntegralClippingBounds()
            );
        }

        public PIDFEx(
                double kp,
                double ki,
                double kd,
                double kf,
                double sp,
                double pv,
                double alpha,
                double deadzone,
                double integralWorkingBounds,
                double integralClippingBounds
        ) {
            kP = kp;
            kI = ki;
            kD = kd;
            kF = kf;

            setPoint = sp;
            measuredValue = pv;

            minIntegral = -1.0;
            maxIntegral = 1.0;

            this.alpha = alpha;

            lastTimeStamp = 0;
            period = 0;

            errorVal_p = setPoint - measuredValue;

            this.integralWorkingBounds = integralWorkingBounds;
            this.integralClippingBounds = integralClippingBounds;

            filter = new IIRFilter(alpha, () -> errorVal_p);

            this.setIntegrationBounds(-integralClippingBounds, integralClippingBounds);

            this.deadzone = deadzone;

            reset();
        }

        public void reset() {
            totalError = 0;
            prevErrorVal_filtered = 0;
            lastTimeStamp = 0;
        }

        public void setTolerance(double positionTolerance) {
            setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
        }

        public void setTolerance(double positionTolerance, double velocityTolerance) {
            errorTolerance_p = positionTolerance;
            errorTolerance_v = velocityTolerance;
        }

        public double getSetPoint() {
            return setPoint;
        }

        public void setSetPoint(double sp) {
            setPoint = sp;
            errorVal_p = setPoint - measuredValue;
            errorVal_p_filtered = filter.get();

            errorVal_v = (errorVal_p_filtered - prevErrorVal_filtered) / period;
        }

        public boolean atSetPoint() {
            return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
        }
        public double[] getCoefficients() {
            return new double[]{kP, kI, kD, kF};
        }

        public double getPositionError() {
            return errorVal_p;
        }

        public double[] getTolerance() {
            return new double[]{errorTolerance_p, errorTolerance_v};
        }

        public double getVelocityError() {
            return errorVal_v;
        }

        public double calculate(double pv) {

            prevErrorVal_filtered = errorVal_p_filtered;

            double currentTimeStamp = (double) System.nanoTime() / 1E9;
            if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
            period = currentTimeStamp - lastTimeStamp;
            lastTimeStamp = currentTimeStamp;

            if (measuredValue == pv) {
                errorVal_p = setPoint - measuredValue;
            } else {
                errorVal_p = setPoint - pv;
                measuredValue = pv;
            }

            errorVal_p_filtered = filter.get();

            if (Math.abs(period) > 1E-6) {
                errorVal_v = (errorVal_p_filtered - prevErrorVal_filtered) / period;
            } else {
                errorVal_v = 0;
            }

            /*
            if total error is the integral from 0 to t of e(t')dt', and
            e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
             */
            if (errorVal_p > -integralWorkingBounds && errorVal_p < integralWorkingBounds) {
                totalError += period * (setPoint - measuredValue);
                totalError = totalError < minIntegral
                        ? minIntegral
                        :
                        Math.min(maxIntegral, totalError);
            } else if (setPoint - measuredValue < errorTolerance_p) {
                totalError = 0;
            }

            // returns u(t)
            return Math.abs(getPositionError()) > deadzone ? kP * errorVal_p +
                    kI * totalError + kD * errorVal_v + kF * setPoint : 0;
        }

        public void setPIDF(double kp, double ki, double kd, double kf) {
            kP = kp;
            kI = ki;
            kD = kd;
            kF = kf;
        }

        public void setIntegrationBounds(double integralMin, double integralMax) {
            minIntegral = integralMin;
            maxIntegral = integralMax;
        }

        public void setCofficients(PIDFExCoeffs cons) {
            setP(cons.getkP());
            setI(cons.getkI());
            setD(cons.getkD());
            setF(cons.getkF());
            setAlpha(cons.getAlpha());
            setWorkingBounds(cons.getIntegralWorkingBounds());
            setIntegralClippingBounds(cons.getIntegralClippingBounds());
        }

        public void clearTotalError() {
            totalError = 0;
        }

        public void setP(double kp) {
            kP = kp;
        }

        public void setI(double ki) {
            kI = ki;
        }

        public void setD(double kd) {
            kD = kd;
        }

        public void setF(double kf) {
            kF = kf;
        }

        public void setAlpha(double newAlpha) {
            alpha = newAlpha;
        }

        public void setWorkingBounds(double bound) {
            integralWorkingBounds = bound;
        }

        public void setIntegralClippingBounds(double bound) {
            integralClippingBounds = bound;
        }

        public double getP() {
            return kP;
        }

        public double getI() {
            return kI;
        }

        public double getD() {
            return kD;
        }

        public double getF() {
            return kF;
        }

        public double getPeriod() {
            return period;
        }

        public double getAlpha() {
            return alpha;
        }

        public double getWorkingBounds() {
            return integralWorkingBounds;
        }
}
