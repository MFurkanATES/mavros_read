# mavros_read
autopilot mavros 

Bu klasorde ROS(robot operation system) üzerinden ardupilot(apm) yazilimi ile calisan otopilotlarla(pixhawk,omnibus,radiolink mini pix) mavros uzerinden bazi sensorlerden veri cekmek icin ornek kod parcalari bulunmaktadir.Bu parcalar kullanilarak kendi kodunuz icin parcaları birlestirerek ardupilot calismalarinizda kullanabilirsiniz.Dosyalarda ki kodlar tek bir topic i  takip etmek icin yazilmistir,bunları tek bir listener() fonksiyonu icerisine yazarak da kullanabilirsiniz.catkin workspace icerisine eklemeden direk indirdiginiz klasorde python komutuyla terminalde calistirip ciktilari gorebilirsiniz.

Kullanilan yazilim ve donanimlar,kodlar bu sistemlerle ve yazilimlarla test edilmistir.

--THİNKPAD T400 

--JETSON NANO 

--JETSON TX2

--UBUNTU 16.04

--UBUNTU 18.04

--ROS KİNETİC

--ROS MELODİC

--PİXHAWK CUBE BLACK

--RADİOLİNK MİNİ PİX V1.0

--OMNİBUS F4 PRO

Bu kodlar ayri ayri terminallerde,yazilmiş siralamayla calistirilmalidir.Konu dahilinde hizli bir baslangic yapabilmeniz için paylasilmistir.

detaylara https://www.ros.org/ ve https://wiki.ros.org/mavros

1.adim,ROS'un calistirilmasi

roscore

2.adim,Mavrosun baslatilmasi 

roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200

burda otopilotunuzun hangi porta bagli oldugunu bilgisayarinizin terminalinde  ls /dev komutunu yazarak bulabilirsiniz,baudrate degeri yer kontrol yazilimindan ayarlanabilir benim bilgisayarlarimda ttyACM0 portuna  baglaniyor seri port haberlesme hizim 115200 bps olarak ayarli

3.adim,ros icerisine yayinlanmamis paketler olabilir,yani dosyalardan birini calistirdiginizda yada kendi düzenlediginiz kodunuzda ulasmak istediginiz verilere ulasamiyorsaniz set_stream_service_ros.py icerisinde ki fonksyonu kendi kodunuza ekleyip ilk basta cagirabilirsiniz yada terminalde rosservice call /mavros/set_stream_rate 0 10 1 komutu yazilarak tüm paketlerin yayinlanmasini saglayacaktır.

İyi ucuslar 

M.Furkan ATES
