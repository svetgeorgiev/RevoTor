                                         // Вывод тока и напряжения в монитор последовательного порта:
                                         //
#include <Battery_Shield.h>              // Подключаем библиотеку Battery_Shield.
Battery_Shield pwrBank;                  // Объявляем объект pwrBank для работы с функциями и методами библиотеки Battery_Shield.
                                         //
void setup(){                            //
    Serial.begin(9600);                  // Инициируем передачу данных в монитор последовательного порта на скорости 9600.
    pwrBank.begin(0.0128f);              // Инициируем работу с Battery Shield, указывая номинал сопротивления (0.128 Ом) установленного в цепи аккумулятора.
}                                        // Номинал сопротивления Вашего источника автономного питания указан на вкладыше к Battery Shield, это значение используется для расчёта Ibat.
                                         //
void loop(){                                                                 // 
    Serial.println("-------------------------");                             // 
    Serial.println((String) "Vbat=" + pwrBank.voltmeter(BATTERY)   + "В.");  // Выводим напряжение аккумулятора в В.
    Serial.println((String) "Ibat=" + pwrBank.amperemeter(BATTERY) + "А.");  // Выводим силу тока  аккумулятора в А.
    Serial.println((String) "Vout=" + pwrBank.voltmeter(OUTPUT)    + "В.");  // Выводим напряжение на выходе    в В.
    Serial.println((String) "Iout=" + pwrBank.amperemeter(OUTPUT)  + "А.");  // Выводим силу тока  на выходе    в А.
    delay(1000);                                                             // Приостанавливаем выполнение скетча на 1 секунду.
}
