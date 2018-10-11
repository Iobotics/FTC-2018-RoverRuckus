package org.firstinspires.ftc.team8740;

import android.app.Application;
import android.content.Context;

/**
 * Created by student on 10/10/2018.
 */

public class ApplicationContextProvider extends Application {
    /**
     * Keeps a reference of the application context
     */
    private static Context sContext;

    @Override
    public void onCreate() {
        super.onCreate();

        sContext = getApplicationContext();

    }

    /**
     * Returns the application context
     *
     * @return application context
     */
    public static Context getContext() {
        return sContext;
    }
}
