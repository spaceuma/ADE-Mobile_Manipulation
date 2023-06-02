***get logs out (bash)***
cp rosbags/1/1_0.db/* .
mv plan.csv path01.txt
mv display_planned_path.csv profile01.txt
mv time.txt computationTime01.txt

***clean path01.txt***
vim path01.txt
:%s/'header'/\r'header'/g
:%s/.*'position': {'x'//g
:%s/, 'y': / /g
:%s/, 'z': / /g
:%s/}, 'ori.*//g
:set ff=unix

***clean profile01.txt***
vim profile01.txt
:%s/'header'/\r'header'/g
:%s/'positions'/\r'positions'/g
:%s/, 'velocities'.*//g
:%s/]//g
:%s/,//g
:set ff=unix
