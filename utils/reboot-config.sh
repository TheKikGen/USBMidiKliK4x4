DEVICE=`amidi -l | grep MidiKl | egrep -o "hw:[0-9]+,0,0"` 

echo Turn device : $DEVICE

amidi -p ${DEVICE} -S "F0 77 77 78 06 08 F7"
