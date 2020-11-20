# Ankara_mavros

Bilgisayarda calıştırmak için önce
ilk terminalde roslaunch mavros apm.launch fcu_url:=/***** komutuyla mavrosu çalıştırın(yıldızlar yerine /dev/ttyACM0:115200 gibi otopilotun tty portlarından hangisine bağlandığını yazmalısınız)
diğer terminalde python kamera_test.py dosyasını calıştırın dahili kamera varsa onu calıştırarak opencv den ros içerisine yayın yapar
3. terminalde Ankara_ros_test.py dosyasını calıştırarak yayınlanan opencv yayınını alarak üzerine otopilottan gelen verileri yazdıracaktır
Örnek çıktı dosyalar arasında var
