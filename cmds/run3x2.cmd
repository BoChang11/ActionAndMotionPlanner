cp bin/Abetare bin/AbetareX

for ai in seq-sat-lama-2011 seq-opt-fdss-2; do
for dims in 3x2; do 
for objs in 4; do
for inst in 1 2 3; do
for(( i = 0; i < 30; i++ )) do
 prob="data/Prob"$dims"_Obj"$objs"_"$inst".txt"
 fname="stats/statProb"$dims"_Obj"$objs"_"$inst$ai"New.txt"
 echo "$fname" 
 touch $fname
 n=`wc -l $fname | cut -f1 -d' '`
 ./bin/AbetareX RunPlanner $prob  $fname 30 120 $ai
 echo "finished run $i"

done
done
done
done
done


