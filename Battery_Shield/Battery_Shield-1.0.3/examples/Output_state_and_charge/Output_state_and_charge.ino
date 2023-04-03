                                         // Вывод состояния и заряда аккумулятора в монитор последовательного порта:
                                         //
#include <Battery_Shield.h>              // Подключаем библиотеку Battery_Shield.
Battery_Shield pwrBank;                  // Объявляем объект pwrBank для работы с функциями и методами библиотеки Battery_Shield.
                                         //
void setup(){                            //
    Serial.begin(9600);                  // Инициируем передачу данных в монитор последовательного порта на скорости 9600.
    pwrBank.begin(0.0128f);              // Инициируем работу с Battery Shield, указывая номинал сопротивления (0.128 Ом) установленного в цепи аккумулятора.
}                                        // Номинал сопротивления Вашего источника автономного питания указан на вкладыше к Battery Shield, это значение используется для расчёта Ibat.
                                         //
void loop(){                             // 
    switch(pwrBank.getState()){          // Выводим текст в зависимости от значения возвращённого функцией getState():
        case CHARGING_IDLE:              Serial.println(       F("Аккумулятор не заряжается.")                                    ); break;
        case CHARGING_TK:                Serial.println(       F("Аккумулятор заряжается малым током.")                           ); break;
        case CHARGING_CC:                Serial.println(       F("Аккумулятор заряжается постоянным током.")                      ); break;
        case CHARGING_CV:                Serial.println(       F("Аккумулятор заряжается постоянным напряжением.")                ); break;
        case CHARGING_TO:                Serial.println(       F("Аккумулятор не заряжается по причине истечения времени заряда.")); break;
        default:                         Serial.println(       F("Режим заряда аккумулятора неизвестен.")                         ); break;
    }                                    Serial.println((String) "Текущая ёмкость аккумулятора " + pwrBank.getLevel() + "%"       );
                                         Serial.println(       F("-------------------------")                                     );
    delay(1000);                         // Приостанавливаем выполнение скетча на 1 секунду.
}                                        //
