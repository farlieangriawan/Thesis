#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <OneWire.h>
#include <Servo.h>

//Defenisikan Wifi,Firebase
#define FIREBASE_HOST "FIREBASEHOST"
#define FIREBASE_AUTH "AUTHENTICATION CODE"
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"

//D2 SSR    D4 SUHU
//D3 SERVO  D5 Switch

#define PWM D2        //pin SSR
#define Switch D5     //pin LimitSwitch

//Defenisikan nilai keanggotaan Nilai PWM
#define high 20
#define mid 10
#define low 0
//Slow Cooker
unsigned long lama2Jam = 7200000;
unsigned long lama4Jam = 14400000;
unsigned long lama6Jam = 21600000;

//Microwave
unsigned long lama1Menit = 60000;
unsigned long lama3Menit = 180000;
unsigned long lama5Menit = 300000;
unsigned long lama10Menit = 600000;
unsigned long lama15Menit = 900000;
unsigned long lama20Menit = 1200000;

unsigned long pre_time;
unsigned long cur_time;

int menit;
float temperature;

OneWire ds(D4);                 //pin sensor suhu DS18B20
Servo servo;

//Ddefenisikan Set Point Suhu
int sp1 = 70;
int sp2 = 80;
int sp3 = 90;
int sp4 = 75;

float error,derror,elow,emid,ehigh,delow,demid,dehigh,temperror;
float sum_alpha,alfa1,alfa2,alfa3,alfa4,alfa5,alfa6,alfa7,alfa8,alfa9;
float sum_z,z1,z2,z3,z4,z5,z6,z7,z8,z9;
float Z;

void setup() 
{
  Serial.begin  (115200);
  pinMode       (PWM, OUTPUT);
  pinMode       (Switch, INPUT);
  digitalWrite  (Switch, HIGH);
  servo.attach  (D3);           //pin Servo
  servo.write(0);
  delay(1000);

//connect ke WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

//connect ke WiFi
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}

void fuzzy2jam()
{
  Serial.println("Memasak Bubur Selama 2 Jam");
  Serial.println();
  delay(500);

  error=(sp3-temperature);
  derror=error-temperror;
  temperror=error;

  Serial.print("  Waktu =  ");
  menit = cur_time/60000;
  Serial.print(menit);     
  Serial.println(" Menit");          
  Serial.println("  ----------------------");
  Serial.print("  Suhu =  ");
  Serial.println(temperature);
  Serial.print("  Error =  ");
  Serial.println(error);
  Serial.print("  DError = ");
  Serial.println(derror);
      
  //errorsuhu
  if (error > 5)
  {
  elow=0;
  emid=0;
  ehigh=1;
  }
    else if ((error>=0)&&(error<=5))
  {
  elow=0;
  emid=((5-error)/5);
  ehigh=((error-0)/5);
  }
    else if((error>=-5)&&(error<=0))
  {
  elow=((0-error)/5);
  emid=((error+5)/5);
  ehigh=0;
  }
     else
        {
          elow=1;
          emid=0; 
          ehigh=0;
        }
          
//deltaerror
  if (derror > 5)
  {
    delow=0;
    demid=0;
    dehigh=1;
  }
     else if ((derror>=0)&&(derror<=5))
     {
       delow=0;
       demid=((5-derror)/5);
       dehigh=((derror-0)/5);
     }
     else if((derror>=-5)&&(derror<=0))
     {
       delow=((0-derror)/5);
       demid=((derror+5)/5);
       dehigh=0;
     }
     else
     {
       delow=1;
       demid=0;
       dehigh=0;
     }
            
  alfa1=min(elow,delow);
  z1=(alfa1*low);
  alfa2=min(elow,demid);
  z2=(alfa2*low);
  alfa3=min(elow,dehigh);
  z3=(alfa3*mid);
  alfa4=min(emid,delow);
  z4=(alfa4*low);
  alfa5=min(emid,demid);
  z5=(alfa5*mid);
  alfa6=min(emid,dehigh);
  z6=(alfa6*mid);
  alfa7=min(ehigh,delow);
  z7=(alfa7*high);
  alfa8=min(ehigh,demid);
  z8=(alfa8*high);
  alfa9=min(ehigh,dehigh);
  z9=(alfa9*high);
     
//defuzzifikasi 
  sum_alpha=alfa1+alfa2+alfa3+alfa4+alfa5+alfa6+alfa7+alfa8+alfa9;
  sum_z=z1+z2+z3+z4+z5+z6+z7+z8+z9;
  Z=sum_z/sum_alpha;
     
  Serial.print("  Sum Z     :  ");
  Serial.println(sum_z);
  Serial.print("  Sum Alpha :  ");
  Serial.println(sum_alpha);
  Serial.print("  PWM       :  ");
  Serial.println(Z);
  Serial.println("  ----------------------");
  Serial.println();
  analogWrite(PWM,Z);
  delay(1000);    
}

void fuzzy4jam()
{
  Serial.println("Memasak Bubur Selama 4 Jam");
  Serial.println();
  delay(500);

  error=(sp2-temperature);
  derror=error-temperror;
  temperror=error;

  Serial.print("  Waktu =  ");
  menit = cur_time/60000;
  Serial.print(menit);     
  Serial.println(" Menit");          
  Serial.println("  ----------------------");
  Serial.print("  Suhu =  ");
  Serial.println(temperature);
  Serial.print("  Error =  ");
  Serial.println(error);
  Serial.print("  DError = ");
  Serial.println(derror);
      
  //errorsuhu
  if (error > 5)
  {
  elow=0;
  emid=0;
  ehigh=1;
  }
    else if ((error>=0)&&(error<=5))
  {
  elow=0;
  emid=((5-error)/5);
  ehigh=((error-0)/5);
  }
    else if((error>=-5)&&(error<=0))
  {
  elow=((0-error)/5);
  emid=((error+5)/5);
  ehigh=0;
  }
     else
        {
          elow=1;
          emid=0; 
          ehigh=0;
        }
          
//deltaerror
  if (derror > 5)
  {
    delow=0;
    demid=0;
    dehigh=1;
  }
     else if ((derror>=0)&&(derror<=5))
     {
       delow=0;
       demid=((5-derror)/5);
       dehigh=((derror-0)/5);
     }
     else if((derror>=-5)&&(derror<=0))
     {
       delow=((0-derror)/5);
       demid=((derror+5)/5);
       dehigh=0;
     }
     else
     {
       delow=1;
       demid=0;
       dehigh=0;
     }
            
  alfa1=min(elow,delow);
  z1=(alfa1*low);
  alfa2=min(elow,demid);
  z2=(alfa2*low);
  alfa3=min(elow,dehigh);
  z3=(alfa3*mid);
  alfa4=min(emid,delow);
  z4=(alfa4*low);
  alfa5=min(emid,demid);
  z5=(alfa5*mid);
  alfa6=min(emid,dehigh);
  z6=(alfa6*mid);
  alfa7=min(ehigh,delow);
  z7=(alfa7*high);
  alfa8=min(ehigh,demid);
  z8=(alfa8*high);
  alfa9=min(ehigh,dehigh);
  z9=(alfa9*high);
     
//defuzzifikasi 
  sum_alpha=alfa1+alfa2+alfa3+alfa4+alfa5+alfa6+alfa7+alfa8+alfa9;
  sum_z=z1+z2+z3+z4+z5+z6+z7+z8+z9;
  Z=sum_z/sum_alpha;
     
  Serial.print("  Sum Z     :  ");
  Serial.println(sum_z);
  Serial.print("  Sum Alpha :  ");
  Serial.println(sum_alpha);
  Serial.print("  PWM       :  ");
  Serial.println(Z);
  Serial.println("  ----------------------");
  Serial.println();
  analogWrite(PWM,Z);
  delay(1000);
}

void fuzzy6jam()
{
  Serial.println("Memasak Bubur Selama 6 Jam");
  Serial.println();
  delay(500);

  error=(sp1-temperature);
  derror=error-temperror;
  temperror=error;

  Serial.print("  Waktu =  ");
  menit = cur_time/60000;
  Serial.print(menit);     
  Serial.println(" Menit");          
  Serial.println("  ----------------------");
  Serial.print("  Suhu =  ");
  Serial.println(temperature);
  Serial.print("  Error =  ");
  Serial.println(error);
  Serial.print("  DError = ");
  Serial.println(derror);
      
  //errorsuhu
  if (error > 5)
  {
  elow=0;
  emid=0;
  ehigh=1;
  }
    else if ((error>=0)&&(error<=5))
  {
  elow=0;
  emid=((5-error)/5);
  ehigh=((error-0)/5);
  }
    else if((error>=-5)&&(error<=0))
  {
  elow=((0-error)/5);
  emid=((error+5)/5);
  ehigh=0;
  }
     else
        {
          elow=1;
          emid=0; 
          ehigh=0;
        }
          
//deltaerror
  if (derror > 5)
  {
    delow=0;
    demid=0;
    dehigh=1;
  }
     else if ((derror>=0)&&(derror<=5))
     {
       delow=0;
       demid=((5-derror)/5);
       dehigh=((derror-0)/5);
     }
     else if((derror>=-5)&&(derror<=0))
     {
       delow=((0-derror)/5);
       demid=((derror+5)/5);
       dehigh=0;
     }
     else
     {
       delow=1;
       demid=0;
       dehigh=0;
     }
            
  alfa1=min(elow,delow);
  z1=(alfa1*low);
  alfa2=min(elow,demid);
  z2=(alfa2*low);
  alfa3=min(elow,dehigh);
  z3=(alfa3*mid);
  alfa4=min(emid,delow);
  z4=(alfa4*low);
  alfa5=min(emid,demid);
  z5=(alfa5*mid);
  alfa6=min(emid,dehigh);
  z6=(alfa6*mid);
  alfa7=min(ehigh,delow);
  z7=(alfa7*high);
  alfa8=min(ehigh,demid);
  z8=(alfa8*high);
  alfa9=min(ehigh,dehigh);
  z9=(alfa9*high);
     
//defuzzifikasi
  sum_alpha=alfa1+alfa2+alfa3+alfa4+alfa5+alfa6+alfa7+alfa8+alfa9;
  sum_z=z1+z2+z3+z4+z5+z6+z7+z8+z9;
  Z=sum_z/sum_alpha;
     
  Serial.print("  Sum Z     :  ");
  Serial.println(sum_z);
  Serial.print("  Sum Alpha :  ");
  Serial.println(sum_alpha);
  Serial.print("  PWM       :  ");
  Serial.println(Z);
  Serial.println("  ----------------------");
  Serial.println();
  analogWrite(PWM,Z);
  delay(1000);    
} 

void microwave()
{
  
  Serial.print("  Waktu =  ");
  menit = cur_time/60000;
  Serial.print(menit);     
  Serial.println(" Menit");
  Serial.print("  Suhu = ");
  Serial.print(temperature);
  Serial.print(" °C");
  Serial.println();
  analogWrite(PWM,255);
  Serial.println("  ----------------------");
  delay(500);
  
}

void normal()
{
  analogWrite(PWM,255);
  Serial.println("Memasak Nasi Normal");
  Serial.println();
  Serial.print("  Suhu = ");
  Serial.print(temperature);
  Serial.print(" °C");
  Serial.println();
  delay(3000);      
}

void menghangatkan()
{ 
   analogWrite(PWM,255);
   Serial.println("Memasak Selsai");
   Serial.println("Warming");
   Serial.println();
   Serial.print("  Suhu = ");
   Serial.print(temperature);
   Serial.print(" °C");
   Serial.println();
   delay(3000);        
}

void loop()
{
//P. Sensor Suhu DS18B20
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  
  if ( !ds.search(addr)) 
  {
    ds.reset_search();
    delay(250);
    return;
  }
 
 
  if (OneWire::crc8(addr, 7) != addr[7]) 
  {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) 
  {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);    // start conversion, with parasite power on at the end  
  delay(1000);
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);       // Read Scratchpad
 
  for ( i = 0; i < 9; i++) 
  {           
    data[i] = ds.read();
  }
 
  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) 
    {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } 
  else 
  {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  }

//Ambil nilai Suhu
  temperature = (float)raw / 16.0;
  
//Kirim data Suhu
  Firebase.setFloat("Suhu",temperature); 

//Ambil data Firebase
  String path ="/";
  FirebaseObject object = Firebase.get(path);
  String Mode = object.getString("mode");
  int Status = object.getInt("Status");
  Firebase.setInt("Status",0);
  
//
   cur_time=millis()-pre_time;
  Serial.println(".......................");
  Serial.print("Waktu =");
  Serial.print(cur_time);
  Serial.println(" Milidetik");
  delay(1000);

//mode warming pada rice cooker
  if(digitalRead(Switch) == LOW)
  {
    pre_time=millis();
    Serial.print("Status = ");
    Serial.println("Warming");
    Serial.println();
    menghangatkan();
    servo.write(0);
    delay(500);
  }

//mode cook pada rice cooker
  if(digitalRead(Switch) == HIGH) 
  {
    Serial.print("Status = ");
    Serial.println("Cooking");
    Serial.println();
 
    //memasak normal ( Menanak Nasi )
    if ( Mode == "Nasi")
    {
      normal();
    }

    //memasak bubur( slow cooker )
    
    else if ( Mode == "2 Jam")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama2Jam )
        {
          fuzzy2jam();
        }
        else
        {
          Firebase.setInt("Status",1);
          servo.write(45);
          delay(500);
        }
    }
    
    else if ( Mode == "4 Jam")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama4Jam )
        {
          fuzzy4jam();
        }
        else
        {
          Firebase.setInt("Status",1);
          servo.write(45);
          delay(500);
        }
    }

    else if ( Mode == "6 Jam")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama6Jam )
        {
          fuzzy6jam();
        }
        else
        {
          Firebase.setInt("Status",1);
          servo.write(45);
          delay(500);
        }
    } 

  //memanaskan (microwave)
else if ( Mode == "1 Menit")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama1Menit )
        {
          microwave();
          Serial.println("Memanaskan Makanan Selama");
          Serial.print(Mode);
        }
        else
        {
          Firebase.setInt("Status",1);
          delay(1000);
          servo.write(45);
          delay(500);
        }
    }

    else if ( Mode == "3 Menit")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama3Menit )
        {
          microwave();
          Serial.println("Memanaskan Makanan Selama");
          Serial.print(Mode);
        }
        else
        {
          Firebase.setInt("Status",1);
          delay(1000);
          servo.write(45);
          delay(500);
        }
    }

    else if ( Mode == "5 Menit")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama5Menit )
        {
          microwave();
          Serial.println("Memanaskan Makanan Selama");
          Serial.print(Mode);
        }
        else
        {
          Firebase.setInt("Status",1);
          delay(1000);
          servo.write(45);
          delay(500);
        }
    }

    else if ( Mode == "10 Menit")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama10Menit )
        {
          microwave();
          Serial.println("Memanaskan Makanan Selama");
          Serial.print(Mode);
        }
        else
        {
          Firebase.setInt("Status",1);
          delay(1000);
          servo.write(45);
          delay(500);
        }
    }

    else if ( Mode == "15 Menit")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama15Menit )
        {
          microwave();
          Serial.println("Memanaskan Makanan Selama");
          Serial.print(Mode);
        }
        else
        {
          Firebase.setInt("Status",1);
          delay(1000);
          servo.write(45);
          delay(500);
        }
    }

    else if ( Mode == "20 Menit")
    {
        cur_time=millis()-pre_time;
        if(cur_time <= lama20Menit )
        {
          microwave();
          Serial.println("Memanaskan Makanan Selama");
          Serial.print(Mode);
        }
        else
        {
          Firebase.setInt("Status",1);
          delay(1000);
          servo.write(45);
          delay(500);
        }
    }
  }
}
