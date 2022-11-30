set -e
if [[ $# -ne 1 ]]; then
	echo "ERROR: The script requires the path to the log directory as the only argument"
	exit 2
fi

logs=$PWD
rm -rf $logs/extractedLog
mkdir extractedLog
cd $logs/$1/execution/MobileManipulation/data
cp *armMotionProfile.txt $logs/extractedLog/armMotionProfile.txt
cp *costMap.txt $logs/extractedLog/costMap.txt
cp *elevationMap.txt $logs/extractedLog/elevationMap.txt
cp *offset.txt $logs/extractedLog/offset.txt
cp *roverPath.txt $logs/extractedLog/roverPath.txt
cp *sdMap.txt $logs/extractedLog/sdMap.txt
cp *slopeMap.txt $logs/extractedLog/slopeMap.txt
cp *traversabilityMap.txt $logs/extractedLog/traversabilityMap.txt
cp *validityMap.txt $logs/extractedLog/validityMap.txt
cp *wristPath.txt $logs/extractedLog/wristPath.txt
cp *3dcostMap $logs/extractedLog/3dcostMap.txt
cd $logs
