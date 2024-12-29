echo "use_depth: $1";

redis-server &

if [ $1 = "--use-depth" ]; then
   echo "starting both rgb and depth cameras"
   python deoxys_camera_node_org.py --camera-ref rs_0 --use-rgb --use-depth --eval --use-rec &
   python deoxys_camera_node_org.py --camera-ref rs_1 --use-rgb --use-depth --eval --use-rec
else
  echo "starting both rgb cameras only"
  python deoxys_camera_node_org.py --camera-ref rs_0 --use-rgb  --eval --use-rec &
  python deoxys_camera_node_org.py --camera-ref rs_1 --use-rgb --eval --use-rec
fi


wait
