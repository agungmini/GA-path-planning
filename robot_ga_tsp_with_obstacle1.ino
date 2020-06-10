#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "stdio.h"
#include "stdlib.h"

#define KANAN 0
#define KIRI  1

#define ECHO    11
#define TRIGGER 12

//functions
void genetic_algorithm(int awal_generateX,int awal_generateY);
void program_jalan();
void timer2_init();
void readSensor();
void kalibrasi();
int kecepatanMotor(int* sensor,int* weight);
void belokanan(int numb_sens,int vLeft,int vRight);
void belokiri(int numb_sens,int vLeft,int vRight);
void detekCabang(int garis,int num_sens,int kecepatan,int breakTime);
void maju(int v_awal);
void driverMotor(int side,int val);
void rem();
void cek_halangan(int posx,int posy,int orient);
boolean read_srf04();

/////////////////////////////////
//array untuk jarak X
unsigned int jarakX[6][5]= {{1,1,1,1,1},
                           {1,1,1,1,1},
                           {1,1,1,1,1},
                           {1,1,1,1,1},
                           {1,1,1,1,1},
                           {1,1,1,1,1}};

//array untuk jarak Y
unsigned int jarakY[5][6]= {{1,1,1,1,1,1},
                           {1,1,1,1,1,1},
                           {1,1,1,1,1,1},
                           {1,1,1,1,1,1},
                           {1,1,1,1,1,1}};

//array untuk state obstacle X
unsigned int obsX[6][5]= {{0,0,0,0,0},
                           {0,0,0,0,0},
                           {0,0,0,0,0},
                           {0,0,0,0,0},
                           {0,0,0,0,0},
                           {0,0,0,0,0}};
                           
//array untuk state obstacle Y
unsigned int obsY[5][6]= {{0,0,0,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,0,0,0,0},
                           {0,0,0,0,0,0}};
//////////////////////////////////

//pin assignments
LiquidCrystal lcd(13,8,7,4,3,2);
#define CWKIRI  6
#define CCWKIRI 5
#define CWKANAN 9
#define CCWKANAN  10

//variables
char buff[33];
int sens[6],cal[6],res[6],lower[6],upper[6];
const int wKi[6]={-50,-33,-16,16,33,50};
const int wKa[6]={50,33,16,-16,-33,-50};
const int v0= 128;
int vKa,vKi;
int pewaktu=0;
int orientasi= 0; //0 utara; 90 timur;180 barat;270 selatan

//variables for genetic algorithm
unsigned int elit_parent[6][2];
unsigned int target[2];
int target_index;
int pos[2]= {0,0};
unsigned int gen_pop_init[2]= {0,0};

ISR(TIMER2_COMPA_vect){
  readSensor();
  pewaktu++;
}

///////////////////////////
boolean read_srf04(){
  long duration;
  float distance;
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance= duration*0.034/2;
  if(distance< 7.0){
    return 1;
  }
  else{
    return 0;
  }
}

int minim(int a,int b){
  if(a>b){
    return b;
  }
  else{
    return a;
  }
}
///////////////////////////

void setup() {
  // put your setup code here, to run once:
  //setup pin pada driver motor
  pinMode(CWKIRI,OUTPUT);
  pinMode(CCWKIRI,OUTPUT);
  pinMode(CWKANAN,OUTPUT);
  pinMode(CCWKANAN,OUTPUT);

  pinMode(TRIGGER,OUTPUT);
  pinMode(ECHO,INPUT);
  
  Serial.begin(9600);
  //Serial.println("kesesatan dimulai");
  
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("load eeprom");
  delay(500);
  for(int i=0;i<6;i++){
    cal[i]= EEPROM.read(i)*3; //membaca eeprom untuk ambang batas sensor
  }
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("loaded");
  delay(1000);
  timer2_init();
  
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff,"dir: %d",orientasi);
  lcd.print(buff);
  lcd.setCursor(0,1);
  sprintf(buff,"pos: (%d,%d)",pos[0],pos[1]);
  lcd.print(buff);
  delay(1000);

  genetic_algorithm(gen_pop_init[0],gen_pop_init[1]);  //fungsi genetik algoritma
  unsigned int mulai= millis();
  program_jalan();  //fungsi jalan
  unsigned int akhir= millis()- mulai;
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff,"time: %d",akhir);
  lcd.print(buff);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void genetic_algorithm(int awal_generateX,int awal_generateY){
  unsigned int pop[5][6][2];
  unsigned int parent[2][6][2];
  unsigned int child[4][6][2];
  int optVal= 28;
  float fn_max; 
  int iterasi=0;
  int index= 0;
  randomSeed(analogRead(0));
  //kosongkan
  for(int i=0;i<5;i++){
    for(int j=0;j<6;j++){
      pop[i][j][0]=0;
      pop[i][j][1]=0;
    }
  }
  //kesesatan dimulai disini
  //bangkitkan populasi
  for(int i=0;i<5;i++){
    /////////////////////////////////
    pop[i][0][0]=awal_generateX;
    pop[i][0][1]=awal_generateY;
    /////////////////////////////////
    int pop_lama[2]={awal_generateX,awal_generateY};
    for(int j=1;j<5;j++){
      int dX= random(0,5);  //awal gene
      int dY= random(0,5);  //awal gene
      while(((dX==pop_lama[0])&&(dY==pop_lama[0]))||((dX==5)&&(dY==5))){
        dX= random(0,5);  //awal gene
        dY= random(0,5);  //awal gene
      }
      pop[i][j][0]=dX;
      pop[i][j][1]=dY;
    }
    pop[i][5][0]=5;
    pop[i][5][1]=5;
  }
  int eucDist= ((5-awal_generateX)+(5-awal_generateY))+1;
  iterasi=0;
  unsigned int start= millis();
  
  while((optVal>eucDist)&&(iterasi< 250)){    
    //menghitung panjang dan fitness
    int l[5];
    float fn[5];
    for(int i=0;i<5;i++){
      l[i]=0;
      int asalX=awal_generateX;
      int asalY=awal_generateY;
      for(int j=1;j<6;j++){
        //////////////////////////////////
        int tujuanX= pop[i][j][0];
        int tujuanY= pop[i][j][1];
        //hitung untuk follow X
        while(asalX!=tujuanX){
          if(asalX<tujuanX){
            l[i]=l[i]+jarakX[asalY][minim(asalX,tujuanX)];
            asalX++;
          }
          else if(asalX>tujuanX){
            l[i]=l[i]+jarakX[asalY][minim(asalX,tujuanX)];
            asalX--;
          }
        }
        //hitung untuk follow Y
        while(asalY!=tujuanY){
          if(asalY<tujuanY){
            l[i]=l[i]+jarakY[minim(asalY,tujuanY)][asalX];
            asalY++;
          }
          else if(asalY>tujuanY){
            l[i]=l[i]+jarakY[minim(asalY,tujuanY)][asalX];
            asalY--;
          }
        }
        ///////////////////////////////////
      }
      fn[i]= 1000/(1+l[i]);
    }
    
    //mencari nilai minimal
    float fn_min;
    optVal=l[0];
    fn_min=fn[0];
    int index=0;
    for(int i=0;i<5;i++){
      if(optVal>=l[i]){
        optVal=l[i];
        fn_max= fn[i];
        index=i;
      }
      if(fn_min>=fn[i]){
        fn_min=fn[i];
      }
    }
    
    //simpan solusi terbaik untuk iterasi berikutnya
    //print untuk tau rute
    for(int i=0;i<6;i++){
      elit_parent[i][0]=pop[index][i][0];
      elit_parent[i][1]=pop[index][i][1];
    }

    float sum=0;
    for(int i=0;i<5;i++){
      fn[i]=fn[i]-fn_min;
      sum= sum+fn[i];
    }

    for(int i=0;i<5;i++){
      fn[i]=(fn[i]*100)/sum;
    }
    
    //roulette
    float lot[5];
    for(int i=0;i<5;i++){
      if(i==0){
        lot[i]=fn[i];
      }
      else{
        lot[i]=lot[i-1]+fn[i];
      }
    }

    //pilih parent
    for(int i=0;i<2;i++){
        float any_numb= random(0,100);
        for(int j=1;j<5;j++){
          if(any_numb<lot[j]){
            for(int k=0;k<6;k++){
              parent[i][k][0]=pop[j][k][0];
              parent[i][k][1]=pop[j][k][1];
            }
          break;
          }
        }
    }
    
    //crossover
    for(int i=0;i<2;i++){
      switch(i){
        case 0:{
          for(int k=0;k<6;k++){
            child[0][k][0]=parent[i][k][0];
            child[0][k][1]=parent[i][k][1];  
            child[2][k][0]=parent[i][k][0];
            child[2][k][1]=parent[i][k][1];
          }
        }
        case 1:{
          for(int k=0;k<6;k++){
            child[1][k][0]=parent[i][k][0];
            child[1][k][1]=parent[i][k][1];
            child[3][k][0]=parent[i][k][0];
            child[3][k][1]=parent[i][k][1];
          }
        }
      }
    }
    
    for(int i=0;i<2;i++){
    //ubah LSB
      child[0][5][i]=parent[1][5][i];
      child[1][5][i]=parent[0][5][i];
      
    //ubah 4 bit LSB
      child[2][5][i]=parent[1][5][i];
      child[3][5][i]=parent[0][5][i];
      child[2][4][i]=parent[1][4][i];
      child[3][4][i]=parent[0][4][i];
      child[2][3][i]=parent[1][3][i];
      child[3][3][i]=parent[0][3][i];
      child[2][2][i]=parent[1][2][i];
      child[3][2][i]=parent[0][2][i];
    }
  
    //mutasi
    for(int i=0;i<4;i++){
      int mutasi_index= random(1,5);
      child[i][5-mutasi_index][0]=random(0,5); //awal gene
      child[i][5-mutasi_index][1]=random(0,5); //awal gene
    }
    
    //swap ke populasi baru
    for(int i=0;i<4;i++){
      for(int j=0;j<6;j++){
        pop[i+1][j][0]=child[i][j][0];
        pop[i+1][j][1]=child[i][j][1];
        pop[0][j][0]=elit_parent[j][0];
        pop[0][j][1]=elit_parent[j][1];
      }
    }
    
    //for(int i=0;i<6;i++){
    //  sprintf(buff,"(%d,%d)",elit_parent[i][0],elit_parent[i][1]);
    //  Serial.print(buff);
    //}
    //Serial.print(eucDist);
    //Serial.print(",");
    //Serial.print(optVal);
    //Serial.print(",");
    //Serial.println(iterasi);
    
    if(iterasi> 248){
      iterasi=0;
      eucDist++;
    }
    iterasi++;
  }

  for(int i=0;i<6;i++){
    sprintf(buff,"(%d,%d)",elit_parent[i][0],elit_parent[i][1]);
    Serial.print(buff);
  }
  Serial.println(optVal);
  
  unsigned int current= millis()- start;
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff,"Dist: %d",optVal);
  lcd.print(buff);
  lcd.setCursor(0,1);
  sprintf(buff,"time: %d",current);
  lcd.print(buff);
  delay(1000);
  target_index=0;
}

void program_jalan(){
  //here something crazy begins
  while(target_index<6){
  //for(target_index=0;target_index<6;target_index++){
    if((pos[0]==5)&&(pos[1]==5)){
      break;
    }
    target[0]= elit_parent[target_index][0];
    target[1]= elit_parent[target_index][1];
    
    //untuk sumbu X
    while(pos[0]!=target[0]){
      if((pos[0]==5)&&(pos[1]==5)){
        break;
      }
      switch(orientasi){
        case 0:{
          if(pos[0]>target[0]){
            belokiri(2,-v0,v0,10);
          }
          else if(pos[0]<target[0]){
            belokanan(3,v0,-v0,10);
          }
        }
          break;
        case 90:{
          if(pos[0]>target[0]){
            belokiri(2,-v0,v0,10);
            belokiri(2,-v0,v0,10);
          }
          else if(pos[0]<target[0]){
            detekCabang(v0,20);  
          }
        }
          break;
        case 180:{
          if(pos[0]>target[0]){
            belokanan(3,v0,-v0,10);
          }
          else if(pos[0]<target[0]){
            belokiri(2,-v0,v0,10);
          }
        }
          break;
        case 270:{
          if(pos[0]>target[0]){
            detekCabang(v0,20);
          }
          else if(pos[0]<target[0]){
            belokanan(3,v0,-v0,10);
            belokanan(3,v0,-v0,10);
          }
        }
          break;  
      }
    }

    //untuk sumbu Y
    while(pos[1]!=target[1]){
      if((pos[0]==5)&&(pos[1]==5)){
        break;
      }
      switch(orientasi){
        case 0:{
          if(pos[1]>target[1]){
            belokanan(3,v0,-v0,10);
            belokanan(3,v0,-v0,10);
          }
          else if(pos[1]<target[1]){
            detekCabang(v0,20);
          }
        }
          break;
        case 90:{
          if(pos[1]>target[1]){
            belokanan(3,v0,-v0,10);
          }
          else if(pos[1]<target[1]){
            belokiri(2,-v0,v0,10);
          }
        }
          break;
        case 180:{
          if(pos[1]>target[1]){
            detekCabang(v0,20);
          }
          else if(pos[1]<target[1]){
            belokiri(2,-v0,v0,10);
            belokiri(2,-v0,v0,10);
          }
        }
          break;
        case 270:{
          if(pos[1]>target[1]){
            belokiri(2,-v0,v0,10);
          }
          else if(pos[1]<target[1]){
            belokanan(3,v0,-v0,10);
          }
        }
          break;  
      }
    }
    target_index++;
  }
}

void timer2_init(){
  TCCR2A= (1<<WGM21); //CTC
  TCCR2B= TCCR2B|(1<<CS22)|(1<<CS21)|(1<<CS20); //prescaler 1024
  OCR2A = 255;
  TIMSK2|= (1<<OCIE2A); //interupsi enable timer 2 compare
}

void readSensor(){
  sens[0]=analogRead(A5);
  sens[1]=analogRead(A4);
  sens[2]=analogRead(A3);
  sens[3]=analogRead(A2);
  sens[4]=analogRead(A1);
  sens[5]=analogRead(A0);
  for(int i=0;i<6;i++){
    if(sens[i]<cal[i])res[i]=0;
    else if(sens[i]>=cal[i])res[i]=1;
  }
}

void kalibrasi(){
  for(int i=0;i<6;i++){
    if(sens[i]< lower[i])lower[i]= sens[i];
    if(sens[i]> upper[i])upper[i]= sens[i];
    cal[i]=lower[i]+((upper[i]-lower[i])/2);
  }
}

int kecepatanMotor(int* sensor,int* weight){
  int temp=0;
  for(int i=0;i<6;i++){
    temp= temp+(sensor[i]*weight[i]);
  }
  return temp;
}

void belokanan(int numb_sens,int vLeft,int vRight,int breakTime){
  pewaktu=0;
  while(pewaktu<breakTime){
    driverMotor(KANAN,vRight);
    driverMotor(KIRI,vLeft);
    Serial.print(pewaktu);
  }
  Serial.println();
  while(!res[numb_sens]){
    driverMotor(KANAN,vRight);
    driverMotor(KIRI,vLeft);
  }
  rem();
  orientasi= (360+orientasi+90)%360;
  delay(100);
  //cek ada halangan?
  cek_halangan(pos[0],pos[1],orientasi);
  
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff,"dir:%2d",orientasi);
  lcd.print(buff);
  lcd.setCursor(7,0);
  sprintf(buff,"tgt:(%d,%d)",elit_parent[target_index][0],elit_parent[target_index][1]);
  lcd.print(buff);
  lcd.setCursor(0,1);
  sprintf(buff,"pos:(%d,%d)",pos[0],pos[1]);
  lcd.print(buff);
}

void belokiri(int numb_sens,int vLeft,int vRight,int breakTime){
  pewaktu=0;
  while(pewaktu<breakTime){
    driverMotor(KANAN,vRight);
    driverMotor(KIRI,vLeft);
    Serial.print(pewaktu);
  }
  Serial.println();
  while(!res[numb_sens]){
    driverMotor(KANAN,vRight);
    driverMotor(KIRI,vLeft);
  }
  rem();
  orientasi=(360+orientasi-90)%360;
  delay(100);
  //cek ada halangan?
  cek_halangan(pos[0],pos[1],orientasi);
  
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff,"dir:%d",orientasi);
  lcd.print(buff);
  lcd.setCursor(7,0);
  sprintf(buff,"tgt:(%d,%d)",elit_parent[target_index][0],elit_parent[target_index][1]);
  lcd.print(buff);
  lcd.setCursor(0,1);
  sprintf(buff,"pos:(%d,%d)",pos[0],pos[1]);
  lcd.print(buff);
}

void detekCabang(int kecepatan,int breakTime){
  while(!(res[0]||res[5])){
    maju(kecepatan);
  }
  
  switch(orientasi){
    case 0: pos[1]++;
      break;
    case 90: pos[0]++;
      break;
    case 180: pos[1]--;
      break;
    case 270: pos[0]--;
      break;
  }
  
  pewaktu=0;
  while(pewaktu<breakTime){
    maju(kecepatan);
    Serial.print(pewaktu);
  }
  Serial.println();
  rem();

  //cek ada halangan?
  cek_halangan(pos[0],pos[1],orientasi);
  
  lcd.clear();
  lcd.setCursor(0,0);
  sprintf(buff,"dir:%d",orientasi);
  lcd.print(buff);
  lcd.setCursor(7,0);
  sprintf(buff,"tgt:(%d,%d)",elit_parent[target_index][0],elit_parent[target_index][1]);
  lcd.print(buff);
  lcd.setCursor(0,1);
  sprintf(buff,"pos:(%d,%d)",pos[0],pos[1]);
  lcd.print(buff);
}

void maju(int v_awal){
  int vKanan,vKiri;
  vKanan= v_awal+ kecepatanMotor(res,wKa);
  vKiri= v_awal+ kecepatanMotor(res,wKi);
  driverMotor(KANAN,vKanan);
  driverMotor(KIRI,vKiri);
}

void driverMotor(int side,int val){
  if(side== KANAN){
    if(val>= 0){
      analogWrite(CWKANAN,abs(val));
      analogWrite(CCWKANAN,0);
    }
    else if(val< 0){
      analogWrite(CWKANAN,0);
      analogWrite(CCWKANAN,abs(val));
    }
  }
  else if(side== KIRI){
    if(val>= 0){
      analogWrite(CWKIRI,abs(val));
      analogWrite(CCWKIRI,0);
    }
    else if(val< 0){
      analogWrite(CWKIRI,0);
      analogWrite(CCWKIRI,abs(val));
    }
  }
}

void rem(){
  driverMotor(KANAN,0);
  driverMotor(KIRI,0);
}

void cek_halangan(int posx,int posy,int orient){
  int state;
  switch(orient){
    case 0: state= obsY[posy][posx];
      break;
    case 90: state= obsX[posy][posx];
      break;
    case 180: state= obsX[posy][posx-1];
      break;
    case 270: state= obsY[posy-1][posx];
      break;
  }

  //lakukan pemindai jarak
  if(!state){
    if(read_srf04()){
    lcd.clear();
    lcd.setCursor(0,0);  
    lcd.print("halangan");
    delay(1000);
    //update tabel
    int a=posx;
    int b=posy;
    switch(orient){
      case 0:
        jarakY[b][a]+=50;
        obsY[b][a]=1;
        break;
      case 90:
        jarakX[b][a]+=50;
        obsX[b][a]=1;
        break;
      case 180:
        jarakX[b][a-1]+=50;
        obsX[b][a-1]=1;
        break;
      case 270:
        jarakY[b-1][a]+=50;
        obsY[b-1][a]=1;
        break;
    }
    print_tabel();
    //hitung GA ulang
    genetic_algorithm(pos[0],pos[1]);
    program_jalan();
    }
  }
}

void print_tabel(){
  Serial.println("JARAK X");
  for(int i=0;i<6;i++){
    for(int j=0;j<5;j++){
      Serial.print(jarakX[i][j]);
      Serial.print(",");
    }
    Serial.println();
  }
  Serial.println("JARAK Y");
  for(int i=0;i<5;i++){
    for(int j=0;j<6;j++){
      Serial.print(jarakY[i][j]);
      Serial.print(",");
    }
    Serial.println();
  }
}

