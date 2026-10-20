package harness;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

/** Print selected CSV columns as an aligned table:  Table <csv> <col,col,...> */
public final class Table {
    public static void main(String[] a) throws Exception {
        final List<String> lines = Files.readAllLines(Paths.get(a[0]));
        final String[] head = lines.get(0).split(",", -1);
        final String[] want = a[1].split(",");
        final int[] idx = new int[want.length];
        for (int i = 0; i < want.length; i++) for (int j = 0; j < head.length; j++) if (head[j].equals(want[i])) idx[i] = j;
        for (String l : lines) {
            final String[] c = l.split(",", -1);
            final StringBuilder sb = new StringBuilder();
            for (int i : idx) sb.append(String.format("%-14s", i < c.length ? c[i] : ""));
            System.out.println(sb);
        }
    }
}
