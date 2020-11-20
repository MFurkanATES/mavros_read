# mavros_read
autopilot mavros 

Bu klasorde ROS(robot operation system) üzerinden ardupilot(apm) yazilimi ile calisan otopilotlarla(pixhawk,omnibus,radiolink mini pix) mavros uzerinden bazi sensorlerden veri cekmek icin ornek kod parcalari bulunmaktadir.Bu parcalar kullanilarak kendi kodunuz icin parcaları birlestirerek ardupilot calismalarinizda kullanabilirsiniz.Dosyalarda ki kodlar tek bir topic i  takip etmek icin yazilmistir,bunları tek bir listener() fonksiyonu icerisine yazarak da kullanabilirsiniz.catkin workspace icerisine eklemeden direk indirdiginiz klasorde python komutuyla terminalde calistirip ciktilari gorebilirsiniz.

Kullanilan yazilim ve donanimlar,kodlar bu sistemlerle ve yazilimlarla test edilmistir.

--THİNKPAD T400 

--JETSON NANO 

--JETSON TX2

--Raspberry Pi 3, 3+, 4

--UBUNTU 16.04

--UBUNTU 18.04

--ROS KİNETİC

--ROS MELODİC

--PİXHAWK CUBE BLACK

--RADİOLİNK MİNİ PİX V1.0

--OMNİBUS F4 PRO

Bu kodlar ayri ayri terminallerde,yazilmiş siralamayla calistirilmalidir.Konu dahilinde hizli bir baslangic yapabilmeniz için türkçe kaynak eksikliği ve kalan kullanıcılarında askeri bilgi diye paylaşmamasına tepki olarak paylasilmistir.Ankara_mavros dosyası bir dronla görüntü işleme üzerinden hedef takibi yapmak için yazılmış ve çalıştırılmıştır.Rc override üzerine pid kontrolle yaptım diye hatırlıyorum. sadece yaw ekseni açık isterseniz diğer eksenlerde de ayarlama yapıp kullanabilirsiniz ama kapalı alanda tehlikeli oluyor :D Not:o dosya kapalı alanda gps ve compasslar kapalı ilken uçuş yapmak için yazılmıştı.Yanlışı doğrusu nedir tam bilmiyorum önümüzdeki yıllarda konu hakkında tecrübem arttığında oturur kuralına göre tekrar yazarım ama şimdilik bu var elimde.Pozisyon  verisiyle yaparsanız benimlede paylaşırsanız sevinirim mail adresim mustafafurkanates@gmail.com .

detaylara  https://www.ros.org/ ve https://wiki.ros.org/mavros adreslerinden ulasabilirsiniz,örnekler sizlere fikir verecektir

1.adim,ROS'un calistirilmasi

roscore

2.adim,Mavrosun baslatilmasi 

roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200

burda otopilotunuzun hangi porta bagli oldugunu bilgisayarinizin terminalinde  ls /dev komutunu yazarak bulabilirsiniz,baudrate degeri yer kontrol yazilimindan ayarlanabilir benim bilgisayarlarimda ttyACM0 portuna  baglaniyor seri port haberlesme hizim 115200 bps olarak ayarli

3.adim,ros icerisine yayinlanmamis paketler olabilir,yani dosyalardan birini calistirdiginizda yada kendi düzenlediginiz kodunuzda ulasmak istediginiz verilere ulasamiyorsaniz set_stream_service_ros.py icerisinde ki fonksyonu kendi kodunuza ekleyip ilk basta cagirabilirsiniz yada terminalde rosservice call /mavros/set_stream_rate 0 10 1 komutu yazilarak tüm paketlerin yayinlanmasini saglayacaktır.

İyi ucuslar 

Bu dosyalar kendi eğlence ve araştırmalarım için hazırlanımış olup ,yarın bir gün uçuşun birinde kaşılaşırsak aklınızda ismimi çağrıştırmak için sizlerle paylaşıma sunulmuştur.Tabi birde üstte yazdıklarım var :D

M.Furkan ATES


