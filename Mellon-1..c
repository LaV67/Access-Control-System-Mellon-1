/*
 * File:   Mellon-1.
 * Rev.2
 * Author: LaV
 * PCB "Mellon - 1"
 * Created on 14 Октябрь 2017 г., 1:51
 * Сomments:Encoding UTF-8
 * Обработка считывателя реализована на прерываниях INT0 и INT1, что позволяет не отслеживать тайминги:
 * каждый импульс низкого уровня по линии D0 вызывает прерывание INT0,а D1 - INT1.
 * По каждому прерыванию происходит запись в массив dat[26]: импульс низкого уровня по D0 - в элемент массива записывается "0",
 * импульс по D1 - "1". Один импульс - одинэлемент массива.
 * Когда считыватель передал всю посылку, i=26, начинается обработка массива:
 * в переменную fcode записываем 1-й байт (биты 1 - 9 (facility карты) в десятичном виде,
 * id_l - младший байт кода карты, id_h - старший. Биты четности не используются.
 * (Функция bin2dec - конвертация из двоичной формы в десятичную.)
 * fcode, id_l, id_h хранятся во внешней EEPROM.
 * Работа с внешней EEPROM через I2C.
 * Для стирания памяти нужно активировать (раскомментировать) функцию I2C_Delete_all().
 * 
 */
#include <xc.h>
#include <p18f452.h>
#pragma config OSC = HS, WDT = OFF, LVP = OFF, CCP2MUX = OFF
#define _XTAL_FREQ 8000000
//------------------------------------------------------------------------------
unsigned char dat [26];//буфер для считывателя
unsigned char fcode = 0;//байт facility карты
unsigned char id_l = 0;//младший байт id карты
unsigned char id_h = 0;//старший байт id карты
unsigned char i = 0;//счетчик бит считывателя
unsigned char card_found = 0;//флаг "карта найдена" 
unsigned char time = 0;
unsigned char time_exit = 0;//время прохода (дверь открыта)
unsigned char err_ack = 0;//ошибка ответа внешней EPPROM
unsigned char err_wcol = 0;//коллизия I2C
unsigned char h_addr = 0;
unsigned char l_addr = 0;
unsigned char buff_I2C = 0;
unsigned char buff1_I2C = 0;
unsigned char buff2_I2C = 0;
unsigned char r = 0;

void init(void){ 
//========================PORTS=================================================
//PORTA-------------------------------------------------------------------------    
    ADCON0bits.ADON = 0;//АЦП выключен
    ADCON1 = 0b00000111;//PORTA цифровой вход
    TRISAbits.RA2 = 0;//Relay
    PORTAbits.RA2 = 1;//Relay on.Реле по умолчанию замкнуто.
    TRISAbits.RA3 = 0;//BeepI. Звуковой сигнализатор на плате
//PORTB-------------------------------------------------------------------------
    TRISBbits.RB0 = 1;//D0 input
    TRISBbits.RB1 = 1;//D1 input
    TRISBbits.RB2 = 0;//LEDGreen.Зеленый светодиод считывателя
    TRISBbits.RB3 = 0;//LEDRed.Красный светодиод считывателя
    TRISBbits.RB4 = 0;//Beep. Звуковой сигнализатор считывателя
    TRISBbits.RB5 = 1;//Exit. Кнопка на выход.
    PORTBbits.RB3 = 1;
//PORTC-------------------------------------------------------------------------    
    TRISCbits.RC2 = 1;//Write. Джампер записи в EEPROM
//PORTD-------------------------------------------------------------------------    
      
//PORTE-------------------------------------------------------------------------    
    TRISEbits.RE0 = 0;//HL3."Power"  
    TRISEbits.RE1 = 0;//HL1."Access OFF"(Доступ запрещен)
    TRISEbits.RE2 = 0;//HL2."Access ON"(Доступ разрешен) 
//===========================Interrupts=========================================    
    GIE = 1;//глобальное разрешение прерывание
    IPEN = 1;//приоритетная системма прерываний
    GIEH = 1;//прерывания с высоким приоритетом
    GIEL = 1;//прерывания с высоким приоритетом
    PEIE = 1;//периферийные прерывания
//INT0,INT1---------------------------------------------------------------------
    //INT0 всегда имеет высокий приоритет
    INT1IP = 1;//высокий приоритет INT1    
       
    INTEDG0 = 0;//прерывание по заднему фронту
    INTEDG1 = 0;
        
    INT0IF = 0;//сброс флага прерывания от INT0
    INT1IF = 0; 
        
    INT0IE = 1;//разрешение прерывания от INT0(RB0)
    INT1IE = 1;
//TMR1--------------------------------------------------------------------------        
    TMR1IE = 1;//разрешить прерывания от TMR1
    TMR1IP = 0;//приоритет низкий
    TMR1CS = 0;//Internal clock (Fosc/4))
    T1CKPS0 = 1;//Предделитель 1:8
    T1CKPS1 = 1;
    TMR1 = 0x0000;//обнуление, счет от 0 до 65535
    TMR1IF = 0;//обнуление флага
    TMR1ON = 1;//активация таймера
//===========================I2C================================================
    //SSPCON1bits.SSPEN = 1;//активация модуля MSSP
    //SSPCON1bits.SSPM3 = 1;//аппаратный режим ведущего I2C
    //SSPCON1 = 0b00101000;//  
    SSPCON1 = 0x28;
    
    SSPCON2 = 0b00000000;
    
    //SSPSTATbits.SMP = 1;//управление длительностью фронта (100кГц или 1МгЦ)
    //SSPSTAT = 0b10000000;
    SSPSTAT = 0x80;
    
    SSPADD = ((_XTAL_FREQ/1000000)/4)-1;//установка битрейта. Тактовая частота 1Мгц
    //SSPADD = 0x4;//((Fosc/BitRate)/4)-1 = ((8MHz/400kHz)/4)-1 = 0x4; Тактовая частота 400кГц
    DDRCbits.RC4 = 1;//SDA
    DDRCbits.RC3 = 1;//SCL
    ACKSTAT=0;//Bit error answer
    WCOL = 0;//Bit collision
//------------------------------------------------------------------------------
}
void I2C_Idle()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)); //проверка состояния линии  
}
void I2C_Start()
{    
    SSPCON2bits.SEN = 1;//формирование стартового бита
    while (SEN==1);   
}
void I2C_ReStart()
{
    SSPCON2bits.RSEN=1;
    while(RSEN==1);//формирование повторно стартового бита
}
void I2C_Stop()
{
    I2C_Idle();
    SSPCON2bits.PEN=1;
    while(PEN==1);//Формирование стопового бита
}
void I2C_NotAck(){
    SSPCON2bits.ACKDT=1, SSPCON2bits.ACKEN=1;
    while (SSPCON2bits.ACKEN);
}
void I2C_AckPoll()
{   //Запрос готовности микросхемы             
    I2C_Start();
    I2C_Idle();
    SSPBUF = 0b10100000;//control byte:1010+000(адресс микросхемы)+0(запись))
    while(BF == 1);//Buffer flag
    I2C_Idle();    
    I2C_Stop(); 
    if(ACKSTAT == 1)err_ack = 1;
    else err_ack = 0;
}
void I2C_AckWait()
{   //ждем готовность slave-микросхемы
    if(err_ack == 1){
        for(char i = 0; i < 100; i++){
        I2C_AckPoll();
        if(err_ack == 0) break;
        }
    }  
}
void I2C_AckChek()
{  //проверка приема slave-микросхемой
    err_ack=0;
    if(ACKSTAT==0) err_ack=0;
    else err_ack = 1;    
}
void I2C_Write_byte(unsigned char h_addr_I2C, unsigned char l_addr_I2C, unsigned char data_I2C)
    {//Запись 1-го байта
        GIE = 0;
        I2C_Start();
        I2C_Idle();
        SSPBUF = 0b10100000;//control byte:1010+000(адресс микросхемы)+0(запись))
        while(BF == 1);//Buffer flag 
        I2C_AckChek(); I2C_Idle(); 
        SSPBUF = h_addr_I2C;//hight address
        while(BF == 1);
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = l_addr_I2C;//low address
        while(BF == 1);
        I2C_AckChek();
        I2C_Idle(); 
        SSPBUF = data_I2C;//data 
        while(BF == 1);
        I2C_AckChek();I2C_Idle();
        I2C_Stop();
        GIE = 1;//!!!Цикл записи до 5мс.!!!
    }
void I2C_Delete_page(unsigned char h_addr_I2C, unsigned char l_addr_I2C)
    {  //удалить(перезаписать 0xFF) страницу (128 байт)     
        GIE = 0;
        I2C_Start();
        I2C_Idle();
        SSPBUF = 0b10100000;//control byte:1010+000(адресс микросхемы)+0(запись))
        while(BF == 1);//Buffer flag 
        I2C_AckChek(); I2C_Idle();       
        SSPBUF = h_addr_I2C;//hight address
        while(BF == 1);
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = l_addr_I2C;//low address
        while(BF == 1);
        I2C_AckChek();
        I2C_Idle(); 
        for(unsigned char i = 0; i < 127; i++){
            SSPBUF = 0xFF;//data 
            while(BF == 1);
            I2C_AckChek();I2C_Idle();            
        }
        I2C_Stop();
        GIE = 1;//!!!Цикл записи до 5мс.!!!
    }
void I2C_Delete_all(){
    for(unsigned char i = 0; i < 255; i++){
        I2C_AckPoll();//проверка готовности микросхемы
        I2C_AckWait();//если занята, - ждем  
        I2C_Delete_page(i,0);//удалить младшую страницу (адрес i,0x00)
        
        I2C_AckPoll();//проверка готовности микросхемы
        I2C_AckWait();//если занята, - ждем  
        I2C_Delete_page(i,128); //удалить старшую страницу (адрес i,128)       
    }    
}
void I2C_Write_3byte(unsigned char h_addr_I2C, unsigned char l_addr_I2C, unsigned char data_I2C, unsigned char data1_I2C, unsigned char data2_I2C)
    {//Запись 3-х байт
        GIE = 0;
        I2C_Start();
        I2C_Idle();
        SSPBUF = 0b10100000;//control byte:1010+000(адресс микросхемы)+0(запись))
        while(BF == 1);//Buffer flag 
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = h_addr_I2C;//hight address
        while(BF == 1);
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = l_addr_I2C;//low address
        while(BF == 1);
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = data_I2C;//data 
        while(BF == 1);
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = data1_I2C;//data1 
        while(BF == 1);
        I2C_AckChek();I2C_Idle(); 
        SSPBUF = data2_I2C;//data2 
        while(BF == 1);
        I2C_Stop();
        GIE = 1;//!!!Цикл записи до 5мс.!!!
    }
void I2C_Write(unsigned char temp_I2C)
{        
    I2C_Idle(); 
    SSPBUF = temp_I2C;//hight address
    while(BF == 1);
    I2C_AckChek();
}
unsigned char I2C_Read(unsigned char h_addr_I2C, unsigned char l_addr_I2C)
    {
      GIE = 0;
      I2C_Start();
      I2C_Idle();
      SSPBUF = 0b10100000;//code byte
      while(BF == 1); 
      I2C_Idle(); 
      SSPBUF = h_addr_I2C;//h address
      while(BF == 1);
      I2C_Idle(); 
      SSPBUF = l_addr_I2C;//l address
      while(BF == 1);
      I2C_Idle();
      
      I2C_ReStart();//read
      SSPBUF = 0b10100001;//control byte:1010+000(адресс микросхемы)+1(запись))
      while(BF == 1); 
      I2C_Idle(); 
      ACKDT = 1;
      RCEN = 1; 
      while(BF==1);
      while(RCEN==1);      
      I2C_NotAck();      
      I2C_Stop;
      ACKDT = 0;
      GIE = 1;
      return SSPBUF;
    }
void I2C_Read_3byte(unsigned char h_addr_I2C, unsigned char l_addr_I2C)
    {
      GIE = 0;
      I2C_Start();
      I2C_Idle();
      SSPBUF = 0b10100000;//code byte
      while(BF == 1); 
      I2C_Idle(); 
      SSPBUF = h_addr_I2C;//h address
      while(BF == 1);
      I2C_Idle(); 
      SSPBUF = l_addr_I2C;//l address
      while(BF == 1);
      I2C_Idle();
      
      I2C_ReStart();//read
      SSPBUF = 0b10100001;//control byte:1010+000(адресс микросхемы)+1(запись))
      while(BF == 1); 
      I2C_Idle(); 
      ACKDT = 0;
      RCEN = 1; 
      while(BF==1); while(RCEN==1);
      buff_I2C = SSPBUF; //читаем 1-байт данных
      ACKEN = 1; while(ACKEN==1); //формирование подтверждения приема мастером
           
      RCEN = 1;
      while(BF==1); while(RCEN==1);
      buff1_I2C = SSPBUF; //читаем 2-й байт данных
      ACKEN=1; while(ACKEN==1);
      
      RCEN = 1;
      while(BF==1); while(RCEN==1);
      buff2_I2C = SSPBUF;//читаем 3-й байт данных
      
      I2C_NotAck();      
      I2C_Stop;
      ACKDT = 0;
      GIE = 1;     
    }
void access_on(void){//Доступ(дверь) открыт
    time_exit = 0;
    RE1 = 0,RE2 = 1, RA2 = 0, RA3 = 0, RB2 = 0, RB4 = 0;
}
void access_off(void){//Доступ закрыт
    RE1 = 1, RE2 = 0, RA2 = 1, RA3 = 0, RB2 = 1, RB4 = 1;
}
void hello(void){
    for(char i = 0; i < 3; i++){
        RE0 = RE1 = RE2 = RA3 = 1;
        time = 0;
        while(time <= 1);//задержка 0.1с    
        RE0 = RE1 = RE2 = RA3 = 0;
        time = 0;
        while(time <= 1);
        time = 0;
    }
    time = 0;
    while(time <= 5);
    access_off();
    PORTEbits.RE0 = 1;//LEDGreenP ON
}
unsigned char ee_read(unsigned char addr){//Чтение из EEPROM
    EEADR = addr;
    EEPGD = 0;
    CFGS = 0;
    RD = 1;
    while(RD)continue;
    return EEDATA;
}
void ee_write(unsigned char addr, unsigned char data){//Запись в EEPROM
    EEADR = addr;
    EEDATA = data;
    EEPGD = 0;
    CFGS = 0;
    WREN = 1;
    GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    WR = 1;
    while(WR)continue;
    GIE = 1;
    WREN = 0;
}
unsigned char pow(unsigned char a){
    if (a == 0) return 1;
    return 2 << (a - 1);
}
unsigned char bin2dec(int a, int b, unsigned char array[]){
    unsigned char n = 0;
    unsigned char dec = 0;
    for (int i = a; i > b; i--){
        if(array[i] == 1) dec = dec + pow(n);
        n++;
        }  
    return dec;
}
void card_search(void){//Поиск карты в EEPROM
    card_found = 0;
    for(int i = 0; i < 255; i=i+3){
        if(fcode == ee_read(i) && id_h == ee_read(i+1) && id_l == ee_read(i+2)){
        card_found = 1;//карта найдена
        break;
        }
    }
}
void card_search_I2C(void){
    /*GIE = 0;//Вариант поиска, чтением по 3 байта.
    card_found = 0;//поиск карты в I2C EEPROM    
    unsigned char h = 0; unsigned char l = 0;    
    while(h <= h_addr && l <= l_addr){
        I2C_AckPoll(); I2C_AckWait();
        I2C_Read_3byte(h, l);        
        if(buff_I2C == fcode && buff1_I2C == id_h && buff2_I2C == id_l){
            card_found = 1;
            break;
        }
        l = l+3; 
        if (l == 255) {l = 0; h++;}
    }
    GIE = 1;*/
    GIE = 0; ////Вариант поиска, чтением по странице (128 байт за раз).
    unsigned char h = 0; int l = 0; card_found = 0;   
    while(h <= h_addr && l <= l_addr){
      I2C_AckPoll(); I2C_AckWait();      
      I2C_Start();
      I2C_Idle();
      SSPBUF = 0b10100000;//code byte
      while(BF == 1); 
      I2C_Idle(); 
      SSPBUF = h;//h address
      while(BF == 1);
      I2C_Idle(); 
      SSPBUF = l;//l address
      while(BF == 1);
      I2C_Idle();
      
      I2C_ReStart();
      SSPBUF = 0b10100001;//control byte:1010+000(адресс микросхемы)+1(запись))
      while(BF == 1); 
      I2C_Idle(); 
      ACKDT = 0;
      for(int i = 0; i < 128; i++){
          RCEN = 1; 
          while(BF==1); while(RCEN==1);
          buff_I2C = SSPBUF; //читаем 1-байт данных
          ACKEN = 1; while(ACKEN==1); //формирование подтверждения приема мастером
           
          RCEN = 1;
          while(BF==1); while(RCEN==1);
          buff1_I2C = SSPBUF; //читаем 2-й байт данных
          ACKEN=1; while(ACKEN==1);
       
          RCEN = 1;
          while(BF==1); while(RCEN==1);
          buff2_I2C = SSPBUF;//читаем 3-й байт данных
      
          if(buff_I2C == fcode && buff1_I2C == id_h && buff2_I2C == id_l){
              card_found = 1;
              break; //если карта найдена, выходим
          }
          ACKEN=1; while(ACKEN==1);
      }      
      l = l +128;
      if(l == 256) l = 0, h++;      
      I2C_NotAck();      
      I2C_Stop;
      ACKDT = 0;
      if(card_found == 1) break;//если карта найдена, выходим
    }
      GIE = 1;     
}
void main(void) {    
    init(); 
    hello(); 
    //I2C_Delete_all();//Функция полного стирания памяти.
    I2C_AckPoll(); I2C_AckWait();
    h_addr = I2C_Read(0xFF,0xFE);
    I2C_AckPoll(); I2C_AckWait();
    l_addr = I2C_Read(0xFF,0xFD);
    if(h_addr == 0xFF && l_addr == 0xFF) h_addr = 0, l_addr = 0;
    GIE = 1;
    while(1){
        if(RB5 == 0)access_on();//нажата кнопка выход             
        if (i >= 26){//обработка считывателя 
            GIE = 0;
            i = 0;
            fcode = bin2dec(8,0,dat);//конвертим facility (элементы 8-1)
            id_l = bin2dec(24,16,dat);//конвертим младший байт idcard (24-17)
            id_h = bin2dec(16,8,dat);//конвертим старший байт idcard (16-9) 
            GIE = 1;
            card_search_I2C();//поиск карты в eeprom
            
            if(RC2 == 0 && card_found == 0){//тампер нажат и карта не найдена, то добавить в eeprom
                I2C_AckPoll(); I2C_AckWait();
                I2C_Write_3byte(h_addr, l_addr, fcode, id_h, id_l);
                l_addr = l_addr+3;
                if(l_addr == 255)l_addr = 0, h_addr++;
                I2C_AckPoll(); I2C_AckWait();
                I2C_Write_3byte(0xFF, 0xFD, l_addr, h_addr, 0x00);//записываем последний занятый адрес
            }            
            if(card_found == 1)access_on();
            if(card_found == 0)access_off();  
            time = 0;
            while(time <= 1)i = 0;
            card_found = 0;
            fcode = 0;
            id_l = 0;
            id_h = 0;
            i = 0;
        }        
    }     
 return;
}
void interrupt high_priority h_ir (void){
//reader1(INT0,INT1)------------------------------------------------------------    
    if (INT0IF == 1){
        INT0IF = 0;
        dat[i] = 0;
        i++; 
    }
    if(INT1IF == 1){
        INT1IF = 0;
        dat[i] = 1;
        i++; 
    }
//------------------------------------------------------------------------------    
}
void interrupt low_priority l_ir (void){   
//время таймера:1/(8000000/4/8/(65535-40535)) = 0,1с.
//8000000Гц - частота кварца;4 такта; 8-коэффициент предделителя;65535-максимальное значение таймера до переполнения; 40535-начало отсчета таймера
    if(TMR1IF == 1)time_exit++, time++; TMR1IF = 0, TMR1 = 40535;
    if(time_exit > 30)access_off(),time_exit = 0;//автоматическиое закрытие двери через 3с  
}


