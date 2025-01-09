package robotcode.util;

import android.graphics.Color;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;
import androidx.core.graphics.ColorUtils;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

public class LABColor {
    public double L, A, B;

    public LABColor(@NonNull NormalizedRGBA color) {
        @ColorInt
        int colorInt = Color.rgb(
                convertToRange(color.red),
                convertToRange(color.green),
                convertToRange(color.blue)
        );

        double[] lab = new double[3];

        ColorUtils.colorToLAB(colorInt, lab);

        L = lab[0];
        A = lab[1];
        B = lab[2];
    }

    public LABColor(double L, double A, double B) {
        this.L = L;
        this.A = A;
        this.B = B;
    }

    public int convertToRange(double value) {
        return Range.clip((int) Math.round(value * 100 * 255), 0, 255);
    }

    public double deltaE(@NonNull LABColor comparing) {
        double[] lab1 = {L, A, B};
        double[] lab2 = {comparing.L, comparing.A, comparing.B};

        return ColorUtils.distanceEuclidean(lab1, lab2);
    }
}
