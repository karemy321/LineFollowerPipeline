import org.junit.runner.*;
import org.junit.runner.notification.Failure;
import java.lang.reflect.Method;
import java.util.concurrent.*;
/** Runs each @Test method of the given classes with a timeout; prints PASS/FAIL/TIMEOUT. */
public class RunEach {
    public static void main(String[] a) throws Exception {
        final long ms = Long.getLong("timeout", 60000);
        int pass = 0, fail = 0;
        for (String cn : a) {
            final Class<?> c = Class.forName(cn);
            for (Method m : c.getMethods()) {
                if (m.getAnnotation(org.junit.Test.class) == null) continue;
                final ExecutorService ex = Executors.newSingleThreadExecutor(r -> { Thread t = new Thread(r); t.setDaemon(true); return t; });
                final Future<Result> f = ex.submit(() -> new JUnitCore().run(Request.method(c, m.getName())));
                String st;
                try {
                    final Result r = f.get(ms, TimeUnit.MILLISECONDS);
                    if (r.wasSuccessful()) { st = "PASS"; pass++; }
                    else { final Failure fl = r.getFailures().get(0); st = "FAIL " + String.valueOf(fl.getMessage()).replace('\n', ' '); fail++; }
                } catch (TimeoutException e) { st = "TIMEOUT"; fail++; f.cancel(true); }
                ex.shutdownNow();
                System.out.println(c.getSimpleName() + "." + m.getName() + ": " + st);
            }
        }
        System.out.println("passed " + pass + ", failed " + fail);
        System.exit(0);
    }
}
