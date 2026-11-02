BEGIN{FS=","}
NR==1{for(i=1;i<=NF;i++)h[$i]=i; next}
{n++; if($h["result"]=="DONE")d++; else if(show) print "  FAIL",$h["track"],$h["scheme"],$h["distract"],$h["c"],$h["x"],$h["result"];
 ml+=$h["meanLat_cm"]; rl+=$h["rmsLat_cm"]; if($h["maxLat_cm"]>mx && $h["result"]=="DONE")mx=$h["maxLat_cm"]; sc+=$h["signChg_per_s"]; sd+=$h["steerStd"]; pj+=$h["powerJerk"]; lp+=$h["lowPower_pct"]; ms+=$h["lib_ms"]; tt+=$h["time_s"]}
END{printf "runs %d done %d | meanLat %.2f rmsLat %.2f worstLat(done) %.1f | signChg/s %.3f steerStd %.1f | powerJerk %.2f lowPower%% %.1f | time %.1f s | lib %.2f ms\n",n,d,ml/n,rl/n,mx,sc/n,sd/n,pj/n,lp/n,tt/n,ms/n}
