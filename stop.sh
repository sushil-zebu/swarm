pkill -9 -f px4
pkill -9 -f gz
pkill -9 -f MicroXRCEAgent

rm -f /tmp/gz*.sock /tmp/.gz*
rm -rf /tmp/gz_transport*