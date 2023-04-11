                                         // Отключение Battery Shield:
                                         //
#include <Battery_Shield.h>              // Подключаем библиотеку Battery_Shield.
Battery_Shield pwrBank;                  // Объявляем объект pwrBank для работы с функциями и методами библиотеки Battery_Shield.
                                         //
void setup(){                            //
    pwrBank.begin(0.0128f);              // Инициируем работу с Battery Shield, указывая номинал сопротивления (0.128 Ом) установленного в цепи аккумулятора.
}                                        // Номинал сопротивления Вашего источника автономного питания указан на вкладыше к Battery Shield, это значение используется для расчёта Ibat.
                                         //
void loop(){                             // 
    if(millis()>5000){ pwrBank.off(); }  // Если прошло более 5 секунд, то отключаем Battery Shield.
}                                        //
