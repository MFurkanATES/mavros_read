import time
class pid(object):

    def __init__(self, ilk_p=0, ilk_i=0, ilk_d=0, ilk_imax=0):
        # default config dosyasi
        self.p_kazanci = ilk_p
        self.i_kazanci = ilk_i
        self.d_kazanci = ilk_d
        self.imax = abs(ilk_imax) # mutlak deger aliyor /integral degerinin maksimum degerini gosteriyor eger hata cok artarsa saturasyon icin 
        self.integrator = 0 # integral toplaminin degeri
        self.son_hata = None
        self.son_yenileme = time.time() 

    # __str__ - pozisyon vektorunu yazdir
    def __str__(self):
       return "P:%s,I:%s,D:%s,IMAX:%s,Integrator:%s" % (self.p_kazanci, self.i_kazanci, self.d_kazanci, self.imax, self.integrator)


    # get_dt -  zaman farkini son yenilemeden aliyor
    def get_dt(self, max_dt):
        simdi = time.time()
        dt = simdi - self.son_yenileme
        self.son_yenileme = simdi
        if dt > max_dt:
            return 0.0
        else:
            return dt

#get_p - p terimini getirir
    def get_p(self, hata):
        return self.p_kazanci * hata

# get_i - i terimini getirir
    def get_i(self, hata, dt):
        self.integrator = self.integrator + hata * self.i_kazanci * dt
        self.integrator = min(self.integrator, self.imax) #iki terimin kucuk olani
        self.integrator = max(self.integrator, -self.imax) #iki terimin buyuk olani
        return self.integrator
# get_d - d terimini getirir
    def get_d(self, hata, dt):
        if self.son_hata is None:
            self.son_hata = hata
        ret = (hata - self.son_hata) * self.d_kazanci * dt
        self.son_hata = hata
        return ret
#pi blogunu cagirir
    def get_pi(self, hata, dt):
        return self.get_p(hata) + self.get_i(hata , dt)

#pd blogunu cagirir
    def get_pd(self , hata , dt):
        return self.get_p(hata) + self.get_d(hata , dt)
#pid blogunu cagirir
    def get_pid(self,hata,dt):
        return self.get_p(hata) + self.get_i(hata , dt) + self.get_d(hata , dt)

#integrali yeniden ayarlamak icin
    def get_integrator(self):
        return self.integrator
#reset_i integrali resetlemek icin
    def reset_i(self):
        self.integrator = 0

    def main(self):
            print "ayarladi"
            #result_p = test_pid.get_p(girdi)
            #result_i = test_pid.get_i(girdi,0.01)
            #result_d = test_pid.get_d(girdi,0.01)
            #result = result_p + result_i + result_d
            #print "Err %s, Result: %f (P:%f, I:%f, D:%f, Int:%f)" % (i, result, result_p, result_i, result_d, self.get_integrator())'''
