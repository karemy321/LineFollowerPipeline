BEGIN{FS=","}
NR>1{ if($7=="NOCAL"){nocal++; print "NOCAL",$0; next}
 n++; ok=($8<=3 && $9<=1 && $10<=10 && $11<=3 && $14==0); if(ok)p++; else if(show) print $2,$3,$4,$5,$6,"dS",$8,$9,"drop",$10,$11,"lost",$14;
 sds+=$9; if($8>mds)mds=$8; sdr+=$11; if($10>mdr)mdr=$10; lost+=$14 }
END{printf "cases %d pass %d nocal %d | meanDS %.2f maxDS %.0f | meanDrop %.2f maxDrop %.0f | lostFrames %d\n",n,p,nocal,sds/n,mds,sdr/n,mdr,lost}
