# Summary of Jitter CSVs: per file, mean over cases (calib=0 and calib=1 separately).
BEGIN { FS="," }
FNR==1 { next }
$6=="NOCAL" || $5=="NOCAL" { nocal[FILENAME]++; next }
{
  k=FILENAME SUBSEP $5
  n[k]++; jS[k]+=$6; jT[k]+=$7; jumps[k]+=$9; hf[k]+=$10; bias[k]+=$11; lost[k]+=$13; if ($12<=5) minp[k]++
  cjS[k]+=$14; cjT[k]+=$15; cjumps[k]+=$17; chf[k]+=$18
  if ($6 > cmax[k]) cmax[k]=$6
}
END {
  printf "%-34s %3s %4s | %-13s %-13s %-11s %-11s %6s %5s %5s\n", "file", "cal", "n", "jitS glare/clean", "jitT glare/clean", "jumps g/c", "hfS g/c", "biasS", "lost", "Tmin"
  for (k in n) { split(k, a, SUBSEP)
    printf "%-34s %3s %4d | %5.2f / %5.2f   %5.2f / %5.2f   %4d / %4d  %5.2f / %5.2f %6.2f %5d %5d\n", a[1], a[2], n[k], jS[k]/n[k], cjS[k]/n[k], jT[k]/n[k], cjT[k]/n[k], jumps[k], cjumps[k], hf[k]/n[k], chf[k]/n[k], bias[k]/n[k], lost[k], minp[k] }
  for (f in nocal) printf "%s: %d NOCAL\n", f, nocal[f]
}
