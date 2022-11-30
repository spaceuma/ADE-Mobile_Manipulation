set -e
if [[ $# -ne 1 ]]; then
	echo "ERROR: The script requires the path to the log directory as the only argument"
	exit 2
fi

logs=$PWD
rm -rf $logs/extractedLog
mkdir extractedLog
cp $logs/$1armMotionProfile.txt $logs/extractedLog/armMotionProfile.txt || true
cp $logs/$1costMap.txt $logs/extractedLog/costMap.txt || true
cp $logs/$1elevationMap.txt $logs/extractedLog/elevationMap.txt || true
cp $logs/$1offset.txt $logs/extractedLog/offset.txt || true
cp $logs/$1roverPath.txt $logs/extractedLog/roverPath.txt || true
cp $logs/$1sdMap.txt $logs/extractedLog/sdMap.txt || true
cp $logs/$1slopeMap.txt $logs/extractedLog/slopeMap.txt || true
cp $logs/$1traversabilityMap.txt $logs/extractedLog/traversabilityMap.txt || true
cp $logs/$1validityMap.txt $logs/extractedLog/validityMap.txt || true
cp $logs/$1wristPath.txt $logs/extractedLog/wristPath.txt || true
cp $logs/$13dcostMap $logs/extractedLog/3dcostMap.txt || true
cp $logs/$1roverPos.txt $logs/extractedLog/roverPos.txt || true
cp $logs/$1armPos.txt $logs/extractedLog/armPos.txt || true
cp $logs/$1armCommand.txt $logs/extractedLog/armCommand.txt || true
cp $logs/$1motionCommand.txt $logs/extractedLog/motionCommand.txt || true
cd $logs
