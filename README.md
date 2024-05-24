# Подсистема оценки обстановки Интеллектуального Агента в среде ROS2

Проект представляет собой реализацию подсистемы оценки обстановки (ПОО) интеллектуального агента (ИА), которая формирует объектное представление о среде.

## Установка

Проект функционировал на виртуальной машине Ubuntu 22.04
1. Установите [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Установите [Eigen 3.4.0](https://gitlab.com/libeigen/eigen/-/releases) - требуется для реализованного фильтра
3. Выполните в терминале следующие команды (не закрывайте терминал):
     - git clone https://github.com/SemyonLityagin/situation_awareness.git
     - cd situation_awareness/diplom_ws
     - source /opt/ros/humble/setup.bash
     - colcon build
     - source install/setup.bash
4. Выполните генерацию сценария с помощью скрипта scenario_gen.py репозитория (ПОО создает ноду в пространстве agent0, поэтому генерируйте с ориентацией на то, что моделируется агент с именем agent_name = agent0)
5. Используйте парсер scen_to_server.py для перевода сценария в вид, читаемый пакетом server
6. Парсер читает файл plan_scenario.json, сформированный генератором, и формурует файл server_data.json
7. Поместите его в директорию diplom_ws\install\server\share\server\scenario
8. В директории diplom_ws\install\sit_awar\share\sit_awar\agent_config лежит файл конфигурации ПОО, где указываются топики для подписки и наименование агента. Сервер отправляет посылки агенту по указанному в этом конфиге имени
9. Вернитесь в терминал пункта 3 (если закрыли - вернитесь в директорию situation_awareness/diplom_ws и пропишите строчки с "source" пункта 3)
10. Запустите ноды командой (в консоли выводятся метрики FPR и TPR для модуля агрегации и фильтрации в рамках такта работы ПОО):
      - ros2 launch server server.launch.py

## Файлы логов

Модули агрегации и фильтрации логгируют метрики TP, FP, TN, FN в файлы logger_agg.txt и logger_fil.txt соответственно в директории diplom_ws. Поместите их в директорию TP_NP_log. Модифицируйте, указав наименования, и используйте скрипт metrics_FPR_TPR.py для построения диаграмм размаха.  
В данной директории на данный момент находятся логи и диаграммы для основного сценария и усложненного.  
Аналогично можно логгировать время работы модулей, но для этого нужно выполнить модификацию файлов diplom_ws\src\sit_awar\src\DataHungarianAggregation.cpp и diplom_ws\src\sit_awar\src\DataKalmanFilter.cpp (нужно закомментировать и раскомментировать строки в коде) и пересобрать проект (пункт 3 Установки без клонирования) 

## TODO
1. Отвязать ПОО от пространства agent0, добавив параметр запуска ноды
2. Продумать более масштабируемую систему привязки к топикам для получения сенсорных посылок
     - возможность использования разных msg файлов в сенсорных посылках
3. Перейти к модификациям фильтра Калмана для работы с нелинейными динамическими системами
4. Рассмотреть теорию суждений для агрегации
5. Упаковать в docker
