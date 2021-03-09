# mavros_read
autopilot mavros 

Bu klasorde ROS(robot operation system) üzerinden ardupilot(apm) yazilimi ile calisan otopilotlarla(pixhawk,omnibus,radiolink mini pix) mavros uzerinden bazi sensorlerden veri cekmek icin ornek kod parcalari bulunmaktadir.Bu parcalar kullanilarak kendi kodunuz icin parcaları birlestirerek ardupilot calismalarinizda kullanabilirsiniz.Dosyalarda ki kodlar tek bir topic i  takip etmek icin yazilmistir,bunları tek bir listener() fonksiyonu icerisine yazarak da kullanabilirsiniz.catkin workspace icerisine eklemeden direk indirdiginiz klasorde python komutuyla terminalde calistirip ciktilari gorebilirsiniz.

Kullanilan yazilim ve donanimlar,kodlar bu sistemlerle ve yazilimlarla test edilmistir.(Linux calıştıran herhangi bir bilgisayar olur yani,temelde ne olduğu önemli değil,yeni başlıyorsanız rahat olun elinizde ki bilgisayar calıştıracaktır sadece dökümanları korkmadan okuyup uygulamaya gayret gösterin)

--THİNKPAD T400 

--THİNKPAD X230

--JETSON NANO 

--JETSON TX2

--Raspberry Pi ZERO, 3, 3+, 4

--UBUNTU 16.04

--UBUNTU 18.04

--UBUNTU 20.04

--ROS KİNETİC

--ROS MELODİC

--ROS NEOTİC

--PİXHAWK CUBE BLACK

--RADİOLİNK MİNİ PİX V1.0

--OMNİBUS F4 PRO

--NAVİO 2

--PXFMİNİ


Bu kodlar ayri ayri terminallerde,yazilmiş siralamayla calistirilmalidir.Konu dahilinde hizli bir baslangic yapabilmeniz için türkçe kaynak eksikliğini azaltmak için paylasilmistir.Ankara_mavros dosyası bir dronla görüntü işleme üzerinden hedef takibi yapmak için yazılmış ve çalıştırılmıştır.Rc override üzerine pid kontrolle yaptım diye hatırlıyorum. sadece yaw ekseni açık isterseniz diğer eksenlerde de ayarlama yapıp kullanabilirsiniz ama kapalı alanda tehlikeli oluyor :D Not:o dosya kapalı alanda gps ve compasslar kapalı ilken uçuş yapmak için yazılmıştı.Yanlışı doğrusu nedir tam bilmiyorum önümüzdeki yıllarda konu hakkında tecrübem arttığında oturur kuralına göre tekrar yazarım ama şimdilik bu var elimde.Pozisyon  verisiyle yaparsanız benimle de paylaşırsanız sevinirim mail adresim mustafafurkanates@gmail.com.



![ankara mavros](https://github.com/MFurkanATES/mavros_read/blob/master/Ankara_mavros-master/frame_screenshot_27.02.2020.png)

detaylara  https://www.ros.org/ ve https://wiki.ros.org/mavros adreslerinden ulasabilirsiniz,örnekler sizlere fikir verecektir

1.adim,ROS'un calistirilmasi

roscore

2.adim,Mavrosun baslatilmasi 

roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:115200

burda otopilotunuzun hangi porta bagli oldugunu bilgisayarinizin terminalinde  ls /dev komutunu yazarak bulabilirsiniz,baudrate degeri yer kontrol yazilimindan ayarlanabilir benim bilgisayarlarimda ttyACM0 portuna  baglaniyor seri port haberlesme hizim 115200 bps olarak ayarli

3.adim,ros icerisine yayinlanmamis paketler olabilir,yani dosyalardan birini calistirdiginizda yada kendi düzenlediginiz kodunuzda ulasmak istediginiz verilere ulasamiyorsaniz set_stream_service_ros.py icerisinde ki fonksyonu kendi kodunuza ekleyip ilk basta cagirabilirsiniz yada terminalde rosservice call /mavros/set_stream_rate 0 10 1 komutu yazilarak tüm paketlerin yayinlanmasini saglayacaktır.

İyi ucuslar 

Bu dosyalar kendi eğlence ve araştırmalarım için hazırlanımış olup ,yarın bir gün uçuşun birinde kaşılaşırsak aklınızda ismimi çağrıştırmak için sizlerle paylaşıma sunulmuştur.Tabi birde üstte yazdıklarım var :D

--ekleme
set_origin.py dosyasi kapalı alanda sistem manipulasyonu içindir.Bu dosya calıştırıldığında otopilotun EKF'si için bir koordinat atar ve 0,0,0 merkez koordinatlarını verir bundan sonra sistemi ros üzerinde hareket ettirip guided moda vs gecirebilirsiniz.Aksi durumda hatırladığım kadarıyla arm etmeyecek ve sizi sinir edecektir.Yer kontrol yazılımınızda yapacağınıs set_ekf_orgin le ayni işi yapar yani.Kılavuzu bir ara komple elden gecirerek yeniden yazacağım daha anlaşılır bir şekilde.Harici olarak bazı videolar ve resimlerle konfigurasyonları ,kurulumları ve başıma gelen sorunların çözümlerini paylaşacağım.

M.Furkan ATES
BlackBird UAV



