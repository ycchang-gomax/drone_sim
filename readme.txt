##
conda activate license_plate_recognition 

##
分開跑不同程式:

path simulator
python app.py

yolo and reactive car simulator
python app_ai.py

##
send ai inference sim data
/ai
python ai_sender_demo.py 


python ai/yolo_deepsort_udp.py --source 0 --backend dshow --width 1280 --height 720 --fps 30 --model yolov8n.pt --classes 2 5 7 --conf 0.25 --show --host 127.0.0.1 --port 47800 --bias_y 120