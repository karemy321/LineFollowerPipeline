import org.junit.runner.JUnitCore;
import org.junit.runner.Request;
import org.junit.runner.Result;
import org.junit.runner.notification.Failure;
/** Run selected test methods: RunSome Class#method ... */
public class RunSome {
    public static void main(String[] a) throws Exception {
        int fail = 0;
        for (String s : a) {
            final String[] p = s.split("#");
            final Result r = new JUnitCore().run(Request.method(Class.forName("com.example.linefollower." + p[0]), p[1]));
            for (Failure f : r.getFailures()) { System.out.println("FAIL " + s + ": " + f.getMessage()); fail++; }
            if (r.wasSuccessful()) System.out.println("ok   " + s);
        }
    }
}
